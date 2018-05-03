// Source file for the scene camera creation program



////////////////////////////////////////////////////////////////////////
// Include files 
////////////////////////////////////////////////////////////////////////

#include "R3Graphics/R3Graphics.h"
#ifdef USE_MESA
#  include "GL/osmesa.h"
#else
#  include "fglut/fglut.h" 
#  define USE_GLUT
#endif

/*#include <Eigen/Dense>
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Vector4d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Matrix;
typedef Matrix<int, Dynamic, Dynamic> MatrixXi;*/
////////////////////////////////////////////////////////////////////////
// Program variables
////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <algorithm>
#include <vector>
#include "tinysplinecpp.h"

// Filename program variables


static char *input_scene_filename = NULL;
static char *input_cameras_filename = NULL;
static char *input_categories_filename = NULL;
static char *output_cameras_filename = NULL;
static char *output_camera_extrinsics_filename = NULL;
static char *output_camera_intrinsics_filename = NULL;
static char *output_camera_names_filename = NULL;
static char *output_nodes_filename = NULL;


// Camera creation variables

static int create_object_cameras = 0;
static int create_room_cameras = 0;
static int create_interior_cameras = 0;
static int create_world_in_hand_cameras = 0;
static int create_path_in_room_cameras = 0;
static int interpolate_camera_trajectory = 0;


// Camera parameter variables

static int width = 640;
static int height = 480;
static double xfov = 0.5; // half-angle in radians
static double eye_height = 1.55;
static double eye_height_radius = 0.05;


// Camera sampling variables

static double position_sampling = 0.07;
static double angle_sampling = RN_PI / 3.0;
static double interpolation_step = 0.1;


// Camera scoring variables

static int scene_scoring_method = 0;
static int object_scoring_method = 0;
static double min_visible_objects = 3;
static double min_visible_fraction = 0.01;
static double min_distance_from_obstacle = 0.05;
static double min_score = 0;


// Rendering variables

static int glut = 1;
static int mesa = 0;


// Informational program variables

static int print_verbose = 0;
static int print_debug = 0;



////////////////////////////////////////////////////////////////////////
// Internal type definitions
////////////////////////////////////////////////////////////////////////

struct Camera : public R3Camera {
public:
  Camera(void) : R3Camera(), name(NULL) {};
  Camera(const Camera& camera) : R3Camera(camera), name((name) ? strdup(name) : NULL) {};
  Camera(const R3Camera& camera, const char *name) : R3Camera(camera), name((name) ? strdup(name) : NULL) {};
  Camera(const R3Point& origin, const R3Vector& towards, const R3Vector& up, RNAngle xfov, RNAngle yfov, RNLength neardist, RNLength fardist)
    : R3Camera(origin, towards, up, xfov, yfov, neardist, fardist), name(NULL) {};
  ~Camera(void) { if (name) free(name); }
  char *name;
};



////////////////////////////////////////////////////////////////////////
// Internal variables
////////////////////////////////////////////////////////////////////////

// State variables

static R3Scene *scene = NULL;
static RNArray<Camera *> cameras;


// Image types

static const int NODE_INDEX_IMAGE = 0;



////////////////////////////////////////////////////////////////////////
// Input/output functions
////////////////////////////////////////////////////////////////////////

static R3Scene *
ReadScene(char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Allocate scene
  scene = new R3Scene();
  if (!scene) {
    fprintf(stderr, "Unable to allocate scene for %s\n", filename);
    return NULL;
  }

  // Read scene from file
  if (!scene->ReadFile(filename)) {
    delete scene;
    return NULL;
  }

  // Remove references and transformations
  scene->RemoveReferences();
  scene->RemoveTransformations();

  // Print statistics
  if (print_verbose) {
    printf("Read scene from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Nodes = %d\n", scene->NNodes());
    printf("  # Lights = %d\n", scene->NLights());
    printf("  # Materials = %d\n", scene->NMaterials());
    printf("  # Brdfs = %d\n", scene->NBrdfs());
    printf("  # Textures = %d\n", scene->NTextures());
    printf("  # Referenced models = %d\n", scene->NReferencedScenes());
    fflush(stdout);
  }

  // Return scene
  return scene;
}



static int
ReadCategories(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read file
  if (!scene->ReadSUNCGModelFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read categories from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    fflush(stdout);
  }

  // Return success
  return 1;
} 



static int
ReadCameras(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  RNScalar neardist = 0.01 * scene->BBox().DiagonalRadius();
  RNScalar fardist = 100 * scene->BBox().DiagonalRadius();
  RNScalar aspect = (RNScalar) height / (RNScalar) width;

  // Open file
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    fprintf(stderr, "Unable to open cameras file %s\n", filename);
    return 0;
  }

  // Read file
  RNScalar vx, vy, vz, tx, ty, tz, ux, uy, uz, xf, yf, value;
  while (fscanf(fp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", &vx, &vy, &vz, &tx, &ty, &tz, &ux, &uy, &uz, &xf, &yf, &value) == (unsigned int) 12) {
    R3Point viewpoint(vx, vy, vz);
    R3Vector towards(tx, ty, tz);
    R3Vector up(ux, uy, uz);
    R3Vector right = towards % up;
    towards.Normalize();
    up = right % towards;
    up.Normalize();
    yf = atan(aspect * tan(xf));
    Camera *camera = new Camera(viewpoint, towards, up, xf, yf, neardist, fardist);
    camera->SetValue(value);
    cameras.Insert(camera);
    camera_count++;
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Read cameras from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count);
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
WriteCameras(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Open file
  FILE *fp = fopen(filename, "w");
  if (!fp) {
    fprintf(stderr, "Unable to open cameras file %s\n", filename);
    return 0;
  }

  // Write file
  for (int i = 0; i < cameras.NEntries(); i++) {
    Camera *camera = cameras.Kth(i);
    R3Point eye = camera->Origin();
    R3Vector towards = camera->Towards();
    R3Vector up = camera->Up();
    fprintf(fp, "%g %g %g  %g %g %g  %g %g %g  %g %g  %g\n",
      eye.X(), eye.Y(), eye.Z(),
      towards.X(), towards.Y(), towards.Z(),
      up.X(), up.Y(), up.Z(),
      camera->XFOV(), camera->YFOV(),
      camera->Value());
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Wrote cameras to %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", cameras.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
WriteCameraExtrinsics(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Open file
  FILE *fp = fopen(filename, "w");
  if (!fp) {
    fprintf(stderr, "Unable to open camera extrinsics file %s\n", filename);
    return 0;
  }

  // Write file
  for (int i = 0; i < cameras.NEntries(); i++) {
    Camera *camera = cameras.Kth(i);
    const R3CoordSystem& cs = camera->CoordSystem();
    R4Matrix matrix = cs.Matrix();
    fprintf(fp, "%g %g %g %g   %g %g %g %g  %g %g %g %g\n",
      matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3], 
      matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3], 
      matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3]);
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Wrote camera extrinsics to %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", cameras.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
WriteCameraIntrinsics(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Open file
  FILE *fp = fopen(filename, "w");
  if (!fp) {
    fprintf(stderr, "Unable to open camera intrinsics file %s\n", filename);
    return 0;
  }

  // Get center of image
  RNScalar cx = 0.5 * width;
  RNScalar cy = 0.5 * height;

  // Write file
  for (int i = 0; i < cameras.NEntries(); i++) {
    Camera *camera = cameras.Kth(i);
    RNScalar fx = 0.5 * width / atan(camera->XFOV());
    RNScalar fy = 0.5 * height / atan(camera->YFOV());
    fprintf(fp, "%g 0 %g   0 %g %g  0 0 1\n", fx, cx, fy, cy);
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Wrote camera intrinsics to %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
WriteCameraNames(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Open file
  FILE *fp = fopen(filename, "w");
  if (!fp) {
    fprintf(stderr, "Unable to open camera names file %s\n", filename);
    return 0;
  }

  // Write file
  for (int i = 0; i < cameras.NEntries(); i++) {
    Camera *camera = cameras.Kth(i);
    fprintf(fp, "%s\n", (camera->name) ? camera->name : "-");
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Wrote camera names to %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
WriteNodeNames(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Open file
  FILE *fp = fopen(filename, "w");
  if (!fp) {
    fprintf(stderr, "Unable to open node name file %s\n", filename);
    return 0;
  }

  // Write file
  for (int i = 0; i < scene->NNodes(); i++) {
    R3SceneNode *node = scene->Node(i);
    const char *name = (node->Name()) ? node->Name() : "-";
    fprintf(fp, "%d %s\n", i+1, name);
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Wrote node names to %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Nodes = %d\n", scene->NNodes());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
WriteCameras(void)
{
  // Write cameras 
  if (output_cameras_filename) {
    if (!WriteCameras(output_cameras_filename)) exit(-1);
  }
  
  // Write camera extrinsics 
  if (output_camera_extrinsics_filename) {
    if (!WriteCameraExtrinsics(output_camera_extrinsics_filename)) exit(-1);
  }
  
  // Write camera intrinsics 
  if (output_camera_intrinsics_filename) {
    if (!WriteCameraIntrinsics(output_camera_intrinsics_filename)) exit(-1);
  }

  // Write camera names 
  if (output_camera_names_filename) {
    if (!WriteCameraNames(output_camera_names_filename)) exit(-1);
  }

  // Write node names
  if (output_nodes_filename) {
    if (!WriteNodeNames(output_nodes_filename)) exit(-1);
  }

  // Return success
  return 1;
}



////////////////////////////////////////////////////////////////////////
// OpenGL image capture functions
////////////////////////////////////////////////////////////////////////

#if 0
static int
CaptureColor(R2Image& image)
{
  // Capture image 
  image.Capture();

  // Return success
  return 1;
}
#endif



static int
CaptureScalar(R2Grid& scalar_image)
{
  // Capture rgb pixels
  unsigned char *pixels = new unsigned char [ 3 * width * height ];
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

  // Fill scalar image
  unsigned char *pixelp = pixels;
  for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
      unsigned int value = 0;
      value |= (*pixelp++ << 16) & 0xFF0000;
      value |= (*pixelp++ <<  8) & 0x00FF00;
      value |= (*pixelp++      ) & 0x0000FF;
      scalar_image.SetGridValue(ix, iy, value);
    }
  }

  // Delete rgb pixels
  delete [] pixels;
  
  // Return success
  return 1;
}



#if 0
static int 
CaptureDepth(R2Grid& image)
{
  // Get viewport dimensions
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  // Get modelview  matrix
  static GLdouble modelview_matrix[16];
  // glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix);
  for (int i = 0; i < 16; i++) modelview_matrix[i] = 0;
  modelview_matrix[0] = 1.0;
  modelview_matrix[5] = 1.0;
  modelview_matrix[10] = 1.0;
  modelview_matrix[15] = 1.0;
  
  // Get projection matrix
  GLdouble projection_matrix[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);

  // Get viewpoint matrix
  GLint viewport_matrix[16];
  glGetIntegerv(GL_VIEWPORT, viewport_matrix);

  // Allocate pixels
  float *pixels = new float [ image.NEntries() ];

  // Read pixels from frame buffer 
  glReadPixels(0, 0, viewport[2], viewport[3], GL_DEPTH_COMPONENT, GL_FLOAT, pixels); 

  // Resize image
  image.Clear(0.0);
  
  // Convert pixels to depths
  int ix, iy;
  double x, y, z;
  for (int i = 0; i < image.NEntries(); i++) {
    if (RNIsEqual(pixels[i], 1.0)) continue;
    if (RNIsNegativeOrZero(pixels[i])) continue;
    image.IndexToIndices(i, ix, iy);
    gluUnProject(ix, iy, pixels[i], modelview_matrix, projection_matrix, viewport_matrix, &x, &y, &z);
    image.SetGridValue(i, -z);
  }

  // Delete pixels
  delete [] pixels;

  // Return success
  return 1;
}
#endif



static void 
DrawNodeWithOpenGL(R3Scene *scene, R3SceneNode *node, R3SceneNode *selected_node, int image_type)
{
  // Set color based on node index
  RNFlags draw_flags = R3_DEFAULT_DRAW_FLAGS;
  if (image_type == NODE_INDEX_IMAGE) {
    draw_flags = R3_SURFACES_DRAW_FLAG;
    unsigned int node_index = node->SceneIndex() + 1;
    unsigned char color[4];
    color[0] = (node_index >> 16) & 0xFF;
    color[1] = (node_index >>  8) & 0xFF;
    color[2] = (node_index      ) & 0xFF;
    glColor3ubv(color);
  }
  
  // Draw elements
  if (!selected_node || (selected_node == node)) {
    for (int i = 0; i < node->NElements(); i++) {
      R3SceneElement *element = node->Element(i);
      element->Draw(draw_flags);
    }
  }

  // Draw references 
  if (!selected_node || (selected_node == node)) {
    for (int i = 0; i < node->NReferences(); i++) {
      R3SceneReference *reference = node->Reference(i);
      reference->Draw(draw_flags);
    }
  }

  // Draw children
  for (int i = 0; i < node->NChildren(); i++) {
    R3SceneNode *child = node->Child(i);
    DrawNodeWithOpenGL(scene, child, selected_node, image_type);
  }
}




static void 
RenderImageWithOpenGL(R2Grid& image, const R3Camera& camera, R3Scene *scene, R3SceneNode *root_node, R3SceneNode *selected_node, int image_type)
{
  // Clear window
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Initialize transformation
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Load camera and viewport
  camera.Load();
  glViewport(0, 0, width, height);

  // Initialize graphics modes  
  glEnable(GL_DEPTH_TEST);

  // Draw scene
  R3null_material.Draw();
  DrawNodeWithOpenGL(scene, root_node, selected_node, image_type);
  R3null_material.Draw();

  // Read frame buffer into image
  CaptureScalar(image);

  // Compensate for rendering background black
  image.Substitute(0, R2_GRID_UNKNOWN_VALUE);
  if (image_type == NODE_INDEX_IMAGE) image.Subtract(1.0);
}



////////////////////////////////////////////////////////////////////////
// Raycasting image capture functions
////////////////////////////////////////////////////////////////////////

static void
RenderImageWithRayCasting(R2Grid& image, const R3Camera& camera, R3Scene *scene, R3SceneNode *root_node,  R3SceneNode *selected_node, int image_type)
{
  // Clear image
  image.Clear(R2_GRID_UNKNOWN_VALUE);

  // Setup viewer
  R2Viewport viewport(0, 0, image.XResolution(), image.YResolution());
  R3Viewer viewer(camera, viewport);

  // Render image with ray casting
  for (int iy = 0; iy < image.YResolution(); iy++) {
    for (int ix = 0; ix < image.XResolution(); ix++) {
      R3Ray ray = viewer.WorldRay(ix, iy);
      R3SceneNode *intersection_node = NULL;
      if (root_node->Intersects(ray, &intersection_node)) {
        if (intersection_node) {
          if (!selected_node || (selected_node == intersection_node)) {
            if (image_type == NODE_INDEX_IMAGE) {
              image.SetGridValue(ix, iy, intersection_node->SceneIndex());
            }
          }
        }
      }
    }
  }
}

  

////////////////////////////////////////////////////////////////////////
// Image capture functions
////////////////////////////////////////////////////////////////////////

static void
RenderImage(R2Grid& image, const R3Camera& camera, R3Scene *scene, R3SceneNode *root_node, R3SceneNode *selected_node, int image_type)
{
  // Check rendering method
  if (glut || mesa) RenderImageWithOpenGL(image, camera, scene, root_node, selected_node, image_type);
  else RenderImageWithRayCasting(image, camera, scene, root_node, selected_node, image_type);
}




////////////////////////////////////////////////////////////////////////
// Camera scoring function
////////////////////////////////////////////////////////////////////////

static RNBoolean
IsObject(R3SceneNode *node)
{
  // Check name
  if (!node->Name()) return 0;
  if (strncmp(node->Name(), "Model#", 6)) return 0;

  // Get category identifier
  R3SceneNode *ancestor = node;
  const char *category_identifier = NULL;
  while (!category_identifier && ancestor) {
    category_identifier = node->Info("empty_struct_obj");
    ancestor = ancestor->Parent();
  }

  // Check category identifier
  if (category_identifier) {
    if (strcmp(category_identifier, "2")) return 0;
  }

  // Passed all tests
  return 1;
}



static RNScalar
ObjectCoverageScore(const R3Camera& camera, R3Scene *scene, R3SceneNode *node = NULL)
{
  // Allocate points
  const int max_npoints = 1024;
  const int target_npoints = 512;
  static R3Point points[max_npoints];
  static int npoints = 0;

  // Check if same node as last time
  // (NOTE: THIS WILL NOT PARALLELIZE)
  static R3SceneNode *last_node = NULL;
  if (last_node != node) {
    last_node = node;
    npoints = 0;

    // Generate points on surface of node
    RNArea total_area = 0;
    for (int j = 0; j < node->NElements(); j++) {
      R3SceneElement *element = node->Element(j);
      for (int k = 0; k < element->NShapes(); k++) {
        R3Shape *shape = element->Shape(k);
        RNScalar area = shape->Area();
        total_area += area;
      }
    }

    // Check total area
    if (RNIsZero(total_area)) return 0;

    // Generate points 
    for (int i = 0; i < node->NElements(); i++) {
      R3SceneElement *element = node->Element(i);
      for (int j = 0; j < element->NShapes(); j++) {
        R3Shape *shape = element->Shape(j);
        if (shape->ClassID() == R3TriangleArray::CLASS_ID()) {
          R3TriangleArray *triangles = (R3TriangleArray *) shape;
          for (int k = 0; k < triangles->NTriangles(); k++) {
            R3Triangle *triangle = triangles->Triangle(k);
            RNScalar area = triangle->Area();
            RNScalar real_nsamples = target_npoints * area / total_area;
            int nsamples = (int) real_nsamples;
            if (RNRandomScalar() < (real_nsamples - nsamples)) nsamples++;
            for (int m = 0; m < nsamples; m++) {
              if (npoints >= max_npoints) break;
              R3Point point = triangle->RandomPoint();
              points[npoints++] = point;
            }
          }
        }
      }
    }
  }

  // Check number of points
  if (npoints == 0) return 0;

  // Count how many points are visible
  int nvisible = 0;
  for (int i = 0; i < npoints; i++) {
    const R3Point& point = points[i];
    R3Ray ray(camera.Origin(), point);
    const RNScalar tolerance_t = 0.01;
    RNScalar max_t = R3Distance(camera.Origin(), point) + tolerance_t;
    RNScalar hit_t = FLT_MAX;
    R3SceneNode *hit_node = NULL;
    if (scene->Intersects(ray, &hit_node, NULL, NULL, NULL, NULL, &hit_t, 0, max_t)) {
      if ((hit_node == node) && (RNIsEqual(hit_t, max_t, tolerance_t))) nvisible++;
    }
  }

  // Compute score as fraction of points that are visible
  RNScalar score = (RNScalar) nvisible/ (RNScalar) npoints;

  // Return score
  return score;
}



static RNScalar
SceneCoverageScore(const R3Camera& camera, R3Scene *scene, R3SceneNode *subtree = NULL, RNBoolean suncg = FALSE)
{
  // Allocate image for scoring
  R2Grid image(width, height);

  // Compute maximum number of pixels in image
  int max_pixel_count = width * height;
  if (max_pixel_count == 0) return 0;

  // Compute minimum number of pixels per object
  int min_pixel_count_per_object = min_visible_fraction * max_pixel_count;
  if (min_pixel_count_per_object == 0) return 0;

  // Render image
  RenderImage(image, camera, scene, scene->Root(), NULL, NODE_INDEX_IMAGE);

  // Allocate buffer for counting visible pixels of nodes
  int *node_pixel_counts = new int [ scene->NNodes() ];
  for (int i = 0; i < scene->NNodes(); i++) node_pixel_counts[i] = 0;
  
  // Log counts of pixels visible on each node
  for (int i = 0; i < image.NEntries(); i++) {      
    RNScalar value = image.GridValue(i);
    if (value == R2_GRID_UNKNOWN_VALUE) continue;
    int node_index = (int) (value + 0.5);
    if ((node_index < 0) || (node_index >= scene->NNodes())) continue;
    node_pixel_counts[node_index]++;
  }

  // Compute score
  RNScalar sum = 0;
  int node_count = 0;
  for (int i = 0; i < scene->NNodes(); i++) {
    R3SceneNode *node = scene->Node(i);
    if (suncg && !IsObject(node)) continue;
    if (subtree && !node->IsDecendent(subtree)) continue;
    if (node_pixel_counts[i] <= min_pixel_count_per_object) continue;
    sum += log(node_pixel_counts[i] / min_pixel_count_per_object);
    node_count++;
  }

  // Compute score (log of product of number of pixels visible in each object)
  RNScalar score = (node_count > min_visible_objects) ? sum : 0;

  // Delete pixel counts
  delete [] node_pixel_counts;

  // Return score
  return score;
}



////////////////////////////////////////////////////////////////////////
// Mask creation functions
////////////////////////////////////////////////////////////////////////

static void
RasterizeIntoZXGrid(R2Grid& grid, R3SceneNode *node,
  const R3Box& world_bbox)
{
  // Check bounding box
  R3Box node_bbox = node->BBox();
  if (!R3Intersects(world_bbox, node_bbox)) return;
  
  // Rasterize elements into grid
  for (int j = 0; j < node->NElements(); j++) {
    R3SceneElement *element = node->Element(j);
    for (int k = 0; k < element->NShapes(); k++) {
      R3Shape *shape = element->Shape(k);
      R3Box shape_bbox = shape->BBox();
      if (!R3Intersects(world_bbox, shape_bbox)) continue;
      if (shape->ClassID() == R3TriangleArray::CLASS_ID()) {
        R3TriangleArray *triangles = (R3TriangleArray *) shape;
        for (int m = 0; m < triangles->NTriangles(); m++) {
          R3Triangle *triangle = triangles->Triangle(m);
          R3Box triangle_bbox = triangle->BBox();
          if (!R3Intersects(world_bbox, triangle_bbox)) continue;
          R3TriangleVertex *v0 = triangle->V0();
          R3Point vp0 = v0->Position();
          R2Point p0(vp0.Z(), vp0.X());
          if (!R2Contains(grid.WorldBox(), p0)) continue;
          R3TriangleVertex *v1 = triangle->V1();
          R3Point vp1 = v1->Position();
          R2Point p1(vp1.Z(), vp1.X());
          if (!R2Contains(grid.WorldBox(), p1)) continue;
          R3TriangleVertex *v2 = triangle->V2();
          R3Point vp2 = v2->Position();
          R2Point p2(vp2.Z(), vp2.X());
          if (!R2Contains(grid.WorldBox(), p2)) continue;
          grid.RasterizeWorldTriangle(p0, p1, p2, 1.0);
        }
      }
    }
  }

  // Rasterize children into grid
  for (int j = 0; j < node->NChildren(); j++) {
    R3SceneNode *child = node->Child(j);
    RasterizeIntoZXGrid(grid, child, world_bbox);
  }
}



static int
ComputeViewpointMask(R3SceneNode *room_node, R2Grid& mask) 
{
  // Get/check room, wall, floor, and ceiling nodes 
  if (!room_node) return 0;
  if (!room_node->Name()) return 0;
  if (strncmp(room_node->Name(), "Room#", 5)) return 0;
  if (room_node->NChildren() < 3) return 0;
  R3SceneNode *floor_node = room_node->Child(0);
  if (!floor_node->Name()) return 0;
  if (strncmp(floor_node->Name(), "Floor#", 6)) return 0;
  R3SceneNode *ceiling_node = room_node->Child(1);
  if (!ceiling_node->Name()) return 0;
  if (strncmp(ceiling_node->Name(), "Ceiling#", 8)) return 0;
  R3SceneNode *wall_node = room_node->Child(2);
  if (!wall_node->Name()) return 0;
  if (strncmp(wall_node->Name(), "Wall#", 5)) return 0;

  // Get bounding boxes in world coordinates
  R3Box room_bbox = room_node->BBox();
  R3Box floor_bbox = floor_node->BBox();
  R3Box ceiling_bbox = ceiling_node->BBox();

  // Get/check grid extent and resolution in world coordinates
  RNScalar grid_sampling_factor = 2;
  RNScalar grid_sample_spacing = min_distance_from_obstacle / grid_sampling_factor;
  if (grid_sample_spacing == 0) grid_sample_spacing = 0.05;
  if (grid_sample_spacing > 0.1) grid_sample_spacing = 0.1;
  R2Box grid_bbox(room_bbox.ZMin(), room_bbox.XMin(), room_bbox.ZMax(), room_bbox.XMax());
  int res1 = (int) (grid_bbox.XLength() / grid_sample_spacing);
  int res2 = (int) (grid_bbox.YLength() / grid_sample_spacing);
  if ((res1 < 3) || (res2 < 3)) return 0;

  // Compute floor mask
  R2Grid floor_mask = R2Grid(res1, res2, grid_bbox);
  RasterizeIntoZXGrid(floor_mask, floor_node, floor_bbox);
  floor_mask.Threshold(0.5, 0, 1);

  // Initialize object mask
  R2Grid object_mask = R2Grid(res1, res2, grid_bbox);
  R3Box object_bbox = room_bbox;
  object_bbox[RN_LO][RN_Y] = floor_bbox[RN_HI][RN_Y] + RN_EPSILON;
  object_bbox[RN_HI][RN_Y] = ceiling_bbox[RN_LO][RN_Y] - RN_EPSILON;

  // Rasterize objects associated with this room into object mask
  for (int i = 0; i < room_node->NChildren(); i++) {
    R3SceneNode *node = room_node->Child(i);
    if ((node == floor_node) || (node == ceiling_node)) continue;
    RasterizeIntoZXGrid(object_mask, node, object_bbox);
  }

  // Rasterize objects associated with no room into object mask
  for (int i = 0; i < room_node->Parent()->NChildren(); i++) {
    R3SceneNode *node = room_node->Parent()->Child(i);
    if (node->NChildren() > 0) continue;
    RasterizeIntoZXGrid(object_mask, node, object_bbox);
  }
  
  // Reverse object mask to by 1 in areas not occupied by objects
  object_mask.Threshold(0.5, 1, 0);

  // Erode object mask to cover viewpoints at least min_distance_from_obstacle
  if (min_distance_from_obstacle > 0) {
    object_mask.Erode(min_distance_from_obstacle / grid_sample_spacing);
  }

  // Combine the two masks
  mask = floor_mask;
  mask.Mask(object_mask);
  
#if 0
  // Debugging
  char buffer[4096];
  sprintf(buffer, "%s_floor_mask.jpg", room_node->Name());
  floor_mask.WriteFile(buffer);
  sprintf(buffer, "%s_object_mask.jpg", room_node->Name());
  object_mask.WriteFile(buffer);
  sprintf(buffer, "%s_mask.jpg", room_node->Name());
  mask.WriteFile(buffer);
#endif
  
  // Return success
  return 1;
}



static int
FindIndexOfRandomPoint(const R2Grid& grid)
{
  // Choose random point in connected component
  int random_point_counter = (int) (RNRandomScalar() * grid.Cardinality());
  for (int i = 0; i < grid.NEntries(); i++) {
    if (grid.GridValue(i) == R2_GRID_UNKNOWN_VALUE) continue;
    if (--random_point_counter == 0) return i;
  }

  // Should not get here
  return -1;
}

/*
static int
SmoothPathBetween(const R2Grid& grid, const Vector2d& init_pos,
                  const Vector3d& init_direction,
                  const Vector2d& end_pos,
                  const Vector3d& end_direction,
                  const ArrayXi& grid,
                  int*path = nullptr,
                  int* path_size = nullptr) {
// Maybe we should take in a Tensor for the grid and a start, end 3D position?
ArrayXi parents_x = ArrayXi::Zero(grid.rows(), grid.cols());
ArrayXi parents_y = ArrayXi::Zero(grid.rows(), grid.cols());
ArrayXd distances = ArrayXd::Zero(grid.rows(), grid.cols());
parents_x -= 1; // Set the initial parent to -1 to know it's uninitialized.
parents_y -= 1;
double max_dist = 100000.0;
distances += max_dist; // Set the initial distances to 'inf'
distances(init_pos.x, init_pos.y) = 0.0;


while (true) {
  double cur_min = max_dist;
  Vector2i cur_min_idx;
  for (int i = 0; i < grid.rows(); ++i) {
    for (int j = 0; j < grid.rows(); ++j) {
        if (distances(i,j) < cur_min) {
            cur_min = distances(i,j);
            cur_min_idx = Vector2i(i,j);
        }
    }
  }
    
  // Iterate over the neighbors:
  for (int ni = -1; ni < 2; ++ni) {
    for (int nj = -1; nj < 2; ++nj) {
      Vector2i offset = Vector2i(ni,nj);
      Vector2i neighbor = cur_min_idx + offset;
      double cur_neighbor_dist = distances(neighbor.x, neighbor.y);
      double alt_dist_to_neighbor = cur_min + offset.norm();
      if (alt_dist_to_neighbor < cur_neighbor_dist) {
        distances(neighbor.x, neighbor.y) = alt_dist_to_neighbor;
        parents_x(neighbor.x, neighbor.y) = cur_min.x;
        parents_y(neighbor.x, neighbor.y) = cur_min.y;
      } 
    }
  }

  
}

}*/



static int
FindIndexOfFurthestPointAlongPath(const R2Grid& grid, int start_index,
  int *path = NULL, int *path_size = NULL)
{
  // Initialize Dijksra bookkeeping
  R2Grid parent_grid(grid), distance_grid(grid);
  parent_grid.Clear(R2_GRID_UNKNOWN_VALUE);
  distance_grid.Clear(FLT_MAX);
  const RNScalar *grid_values = distance_grid.GridValues();
  int neighbor_index, ix, iy;
  int end_index = -1;
  
  // Compute shortest path to all nonzero points
  RNHeap<const RNScalar *> heap(0);
  distance_grid.SetGridValue(start_index, 0);
  parent_grid.SetGridValue(start_index, start_index);
  heap.Push(grid_values + start_index);
  while (!heap.IsEmpty()) {
    const RNScalar *grid_entry = heap.Pop();
    int grid_index = grid_entry - grid_values;
    end_index = grid_index;
    grid.IndexToIndices(grid_index, ix, iy);
    for (int dx = -1; dx <= 1; dx++) {
      int nx = ix + dx;
      if ((nx < 0) || (nx > grid.XResolution()-1)) continue;
      for (int dy = -1; dy <= 1; dy++) {
        int ny = iy + dy;
        if ((ny < 0) || (ny > grid.YResolution()-1)) continue;
        grid.IndicesToIndex(nx, ny, neighbor_index);
        if (neighbor_index == grid_index) continue;
        if (grid.GridValue(neighbor_index) != R2_GRID_UNKNOWN_VALUE) {
          // Kgenova: Determine if this link is valid:
          if (dx != 0 && dy != 0) {
            
            int vertical_idx;
            grid.IndicesToIndex(ix, ny, vertical_idx);
            int horizontal_idx;
            grid.IndicesToIndex(nx, iy, horizontal_idx);
            bool blocked_vertically = grid.GridValue(vertical_idx) == R2_GRID_UNKNOWN_VALUE;
            bool blocked_horizontally = grid.GridValue(horizontal_idx) == R2_GRID_UNKNOWN_VALUE;
            if (blocked_vertically || blocked_horizontally) { continue; }
          }
          
          RNScalar d = distance_grid.GridValue(grid_index) + sqrt(dx*dx + dy*dy);
          RNScalar old_d = distance_grid.GridValue(neighbor_index);
          if (d < old_d) {
            distance_grid.SetGridValue(neighbor_index, d);
            parent_grid.SetGridValue(neighbor_index, grid_index);
            if (old_d == FLT_MAX) heap.Push(grid_values + neighbor_index);
            else heap.Update(grid_values + neighbor_index);
          }
        }
      }
    }
  }

  // Fill in path
  if (path && path_size) {
    // Construct path backwards
    int count = 0;
    path[count++] = end_index;
    while (path[count-1] != start_index) {
      path[count] = (int) parent_grid.GridValue(path[count-1]);
      count++;
    }

    // Reverse order of path
    for (int i = 0; i < count/2; i++) {
      int swap = path[i];
      path[i] = path[count-1-i];
      path[count-1-i] = swap;
    }

    // Fill in path size
    *path_size = count;
  }

  // Return last visited index
  return end_index;
}



////////////////////////////////////////////////////////////////////////
// Camera creation functions
////////////////////////////////////////////////////////////////////////

static void
CreateObjectCameras(void)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  RNScalar neardist = 0.01 * scene->BBox().DiagonalRadius();
  RNScalar fardist = 100 * scene->BBox().DiagonalRadius();
  RNScalar aspect = (RNScalar) height / (RNScalar) width;
  RNAngle yfov = atan(aspect * tan(xfov));
  
  // Create camera with close up view of each object
  for (int i = 0; i < scene->NNodes(); i++) {
    R3SceneNode *node = scene->Node(i);
    if (!node->Name()) continue;
    if (!IsObject(node)) continue;
    if (node->NElements() == 0) continue;
    R3Camera best_camera;

    // Get node's centroid and radius in world coordinate system
    R3Point centroid = node->BBox().Centroid();
    RNScalar radius = node->BBox().DiagonalRadius();

    // Check lots of directions
    int nangles = (int) (RN_TWO_PI / angle_sampling + 0.5);
    RNScalar angle_spacing = (nangles > 1) ? RN_TWO_PI / nangles : RN_TWO_PI;
    for (int j = 0; j < nangles; j++) {
      // Determine view direction
      R3Vector view_direction(-1, 0, 0); 
      view_direction.YRotate((j+RNRandomScalar()) * angle_spacing);
      view_direction.Normalize();

      // Determine camera viewpoint
      RNScalar min_distance = radius;
      RNScalar max_distance = 1.5 * radius/tan(xfov);
      if (min_distance < min_distance_from_obstacle) min_distance = min_distance_from_obstacle;
      if (max_distance < min_distance_from_obstacle) max_distance = min_distance_from_obstacle;
      R3Point viewpoint = centroid - max_distance * view_direction;

      // Project camera viewpoint onto eye height plane (special for planner5d)
      if (node->Parent() && node->Parent()->Parent()) {
        if (node->Parent()->Parent()->Name()) {
          if (strstr(node->Parent()->Parent()->Name(), "Room") || strstr(node->Parent()->Parent()->Name(), "Level")) {
            R3Point floor = node->Parent()->Parent()->Centroid();
            floor[1] = node->Parent()->Parent()->BBox().YMin();
            viewpoint[1] = floor[1] + eye_height;
            viewpoint[1] += 2.0*(RNRandomScalar()-0.5) * eye_height_radius;
          }
        }
      }

      // Ensure centroid is not occluded
      R3Vector back = viewpoint - centroid;
      back.Normalize();
      R3Ray ray(centroid, back);
      RNScalar hit_t = FLT_MAX;
      if (scene->Intersects(ray, NULL, NULL, NULL, NULL, NULL, &hit_t, min_distance, max_distance)) {
        viewpoint = centroid + (hit_t - min_distance_from_obstacle) * back;
      }

      // Compute camera
      R3Vector towards = centroid - viewpoint;
      towards.Normalize();
      R3Vector right = towards % R3posy_vector;
      right.Normalize();
      R3Vector up = right % towards;
      up.Normalize();
      R3Camera camera(viewpoint, towards, up, xfov, yfov, neardist, fardist);

      // Compute score for camera
      camera.SetValue(ObjectCoverageScore(camera, scene, node));
      if (camera.Value() == 0) continue;
      if (camera.Value() < min_score) continue;
                            
      // Remember best camera
      if (camera.Value() > best_camera.Value()) {
        best_camera = camera;
      }
    }

    // Insert best camera
    if (best_camera.Value() > 0) {
      char camera_name[1024];
      const char *node_name = (node->Name()) ? node->Name() : "-";
      const char *parent_name = (node->Parent() && node->Parent()->Name()) ? node->Parent()->Name() : "-";
      sprintf(camera_name, "%s#%s", parent_name, node_name);
      if (print_debug) printf("%s %g\n", camera_name, best_camera.Value());
      Camera *camera = new Camera(best_camera, camera_name);
      cameras.Insert(camera);
      camera_count++;
    }
  }

  // Print statistics
  if (print_verbose) {
    printf("Created object cameras ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count++);
    fflush(stdout);
  }
}



static void
CreateRoomCameras(void)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  RNScalar neardist = 0.01 * scene->BBox().DiagonalRadius();
  RNScalar fardist = 100 * scene->BBox().DiagonalRadius();
  RNScalar aspect = (RNScalar) height / (RNScalar) width;
  RNAngle yfov = atan(aspect * tan(xfov));

  // Create one camera per direction per room 
  for (int i = 0; i < scene->NNodes(); i++) {
    R3SceneNode *room_node = scene->Node(i);
    if (!room_node->Name()) continue;
    if (strncmp(room_node->Name(), "Room#", 5)) continue;

    // Compute viewpoint mask
    R2Grid viewpoint_mask;
    if (!ComputeViewpointMask(room_node, viewpoint_mask)) continue;

    // Sample directions
    int nangles = (int) (RN_TWO_PI / angle_sampling + 0.5);
    RNScalar angle_spacing = (nangles > 1) ? RN_TWO_PI / nangles : RN_TWO_PI;
    for (int j = 0; j < nangles; j++) {
      // Choose one camera for each direction in each room
      R3Camera best_camera;

      // Sample positions 
      R3Box room_bbox = room_node->BBox();
      for (RNScalar z = room_bbox.ZMin(); z < room_bbox.ZMax(); z += position_sampling) {
        for (RNScalar x = room_bbox.XMin(); x < room_bbox.XMax(); x += position_sampling) {
          // Compute position
          R2Point position(x + position_sampling*RNRandomScalar(), z + position_sampling*RNRandomScalar());

          // Check viewpoint mask
          R2Point viewpoint_mask_position(position[1], position[0]); // ZX          
          RNScalar viewpoint_mask_value = viewpoint_mask.WorldValue(viewpoint_mask_position);
          if (viewpoint_mask_value < 0.5) continue;

          // Compute height
          RNScalar y = room_bbox.YMin() + eye_height;
          y += 2.0*(RNRandomScalar()-0.5) * eye_height_radius;
          if (y > room_bbox.YMax()) continue;

          // Compute direction
          RNScalar angle = (j+RNRandomScalar()) * angle_spacing;
          R2Vector direction = R2posx_vector;
          direction.Rotate(angle);
          direction.Normalize();

          // Compute camera
          R3Point viewpoint(position[0], y, position[1]);
          R3Vector towards(direction.X(), -0.2, direction.Y());
          towards.Normalize();
          R3Vector right = towards % R3posy_vector;
          right.Normalize();
          R3Vector up = right % towards;
          up.Normalize();
          R3Camera camera(viewpoint, towards, up, xfov, yfov, neardist, fardist);

          // Compute score for camera
          camera.SetValue(SceneCoverageScore(camera, scene, room_node, TRUE));
          if (camera.Value() == 0) continue;
          if (camera.Value() < min_score) continue;

          // Remember best camera
          if (camera.Value() > best_camera.Value()) {
            best_camera = camera;
          }
        }
      }

      // Insert best camera for direction in room
      if (best_camera.Value() > 0) {
        if (print_debug) printf("ROOM %s %d : %g\n", room_node->Name(), j, best_camera.Value());
        char name[1024];
        sprintf(name, "%s_%d", room_node->Name(), j);
        Camera *camera = new Camera(best_camera, name);
        cameras.Insert(camera);
        camera_count++;
      }
    }
  }
        
  // Print statistics
  if (print_verbose) {
    printf("Created room cameras ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count++);
    fflush(stdout);
  }
}

R3Vector slerp(const R3Vector& v1, const R3Vector& v2, float t/*, float omega*/) {
  /* Assumes the vectors are normalized. */
  float cos_omega = v1.Dot(v2);
  float omega = acos(cos_omega);
  float sin_omega = sin(omega);
  //float sin_omega = sqrt(1.0 - cos_omega*cos_omega);
  float c1 = sin((1.0 - t) * omega) / sin_omega;
  float c2 = sin(t * omega) / sin_omega;
  return c1 * v1 + c2 * v2;
}

std::vector<R3Camera> ConvertTrajectoryToGAPSFormat(const std::vector<float>& trajectory,
  float yfov, float neardist, float fardist) {
  std::vector<R3Camera> output;
  output.reserve(trajectory.size() / 6);
  for (size_t i = 0; i < trajectory.size() / 6; ++i) {
    R3Point position(trajectory[6*i], trajectory[6*i+1], trajectory[6*i+2]);
    R3Vector towards(trajectory[6*i+3], trajectory[6*i+4], trajectory[6*i+5]);
    
    towards.Normalize();
    R3Vector right = towards % R3posy_vector;
    right.Normalize();
    R3Vector up = right % towards;
    up.Normalize();
    R3Camera best_camera(position, towards, up, xfov, yfov, neardist, fardist);
    output.push_back(best_camera); 
  }
  return output;
}


// InterpolateKeypoints creates a continuous trajectory of GAPS cameras from 
// sparse target keypoints and a point of interest.
// [keypoints] Is a vector containing a 2D [keypoint_count,3] array of the 3D camera keypoints to 
//   interpolate, where keypoint_count is the number of keypoints in the sequence.
//   (keypoints[3*i], keypoints[3*i+1], keypoints[3*i+2]) should contain the XYZ
//   coordinates of the i-th keypoint in world space.
// [lookat_point] contains the XYZ world space coordinate of the point of interest.
// [lookat_weight] is the importance of the lookat_point in determining the
//   final direction vector at each point in the trajectory. Formally, is the
//   slerp coefficient from the the forward direction to the the vector from the
//   current world position to lookat_point.
// [downward_tilt] is a float in the range [0,1] dictating how much of a downward
//   tilt to apply. It is the slerp coefficient from the tangent to the B-Spline to
//   the world down direction at each step. The output is the 'forward' direction 
//   before accounting for interest points.
// [speed] is the speed of the camera in (world units)/second for the trajectory.
//   This speed is not guaranteed because position along the curve is interpolated
//   with a greedy first order taylor expansion algorithms. Accuracy will fall
//   off as targeted speed increased. 
// [fps] is the desired frames per second in the trajectory 'video' generated.
// [start_time] The fractional point along the (length-normalized) curve defined
//   by the keypoints at which to start the trajectory.
// [end_time] The fractional point along the (length-normalized) curve defined
//   by the keypoints at which to end the trajectory.
// [verbose] Determines whether debug messages should be printed to stdout.
//
// Returns:
// float = [curve_length] Is the length of the continuous trajectory in world space units.
// [trajectory] The output trajectory. Has shape [frame_count,6]. Each tuple is [XYZ Nx Ny Nz].
//   This output can be converted to the GAPS format with ConvertTrajectoryToR3CameraSequence().
//
// See CreatePathInRoomCameras() for an example of how to use this function.
//
// TODO:
//   1. Add support for multiple interest points.
//
float InterpolateKeypoints(const std::vector<double>& keypoints, 
    const float lookat_point[3], float lookat_weight, float downward_tilt,
    float speed, float fps, std::vector<float>* trajectory) {
  tinyspline::BSpline spline(keypoints.size()/3, 3, 3, TS_CLAMPED);
  std::vector<double> ctrlp = spline.ctrlp();
  std::copy(keypoints.begin(), keypoints.end(), ctrlp.begin()); 
  spline.setCtrlp(ctrlp);
 
  std::vector<double> result;

  // Approximate the total curve length.
  // We need a lot of samples because the curve is nonlinear in u.
  int length_samples = 1000;
  std::array<float,3> last_p = { 0.0, 0.0, 0.0 };
  std::array<float,3> cur_p = last_p;
  float curve_length = 0.0;
  for (int i = 0; i < length_samples; ++i) {
    float u = static_cast<float>(i) / (static_cast<float>(length_samples) - 1.0f);
    result = spline.evaluate(u).result();
    for (int j = 0; j < 3; ++j) {
      cur_p[j] = static_cast<float>(result[j]);
    }
    if (i > 0) {
      float segment_length_sq = 0.0;
      for (int j = 0; j < 3; ++j) { 
        segment_length_sq += (last_p[j] - cur_p[j])*(last_p[j] - cur_p[j]);
      }
      curve_length += sqrt(segment_length_sq);
    }
    last_p = cur_p;
  }

  tinyspline::BSpline beziers = spline.derive().toBeziers();
  // Compute the number of steps to take from the curve length:
  int total_frame_count  = static_cast<int>(fps * curve_length / speed);
  // We want the samples to be evenly spaced in world space, but
  // world position is not a linear function of u. So at each step,
  // take a left-handed estimate of the derivative to compute the update
  // to u for the next sample.
  float u = 0.0f;
  trajectory->reserve(6*total_frame_count);
  for (int i = 0; i < total_frame_count; ++i) {
    result = beziers.evaluate(u).result();
    float derivative_at_u = sqrt(result[0]*result[0] +
        result[1]*result[1] + result[2]*result[2]); // in meters / unit u
    // We have a target a speed measured in meters / sec. 
    // So speed / derivative is in  units u / sec;
    // And seconds per frame * (speed / deriv) is in  units u / frame.
    // (seconds/frame) = (1/fps). 
    // So the final equation is: speed / (fps * derivative)
    // This gives us a first order guess of the current sample point.
    u += speed / (fps * derivative_at_u); 
    // The curve isn't defined outside [0,1] but our approximation might
    // bring us there:
    if (u > 1.0f) { u = 1.0f; }
    // Compute the camera at u
    result = spline.evaluate(u).result();
    R3Point viewpoint(result[0], result[1], result[2]);
    // Compute tangent direction to the path at u.
    // Technically we could cache this for the next loop,
    // but it is unlikely to make a big difference.
    result = beziers.evaluate(u).result();
    R3Vector towards(result[0], result[1], result[2]);
    towards.Normalize();
    // Slerp downward according to the given coefficient
    R3Vector down(0.0, -1.0, 0.0);
    towards = slerp(towards, down, downward_tilt);
    // We assume the lookat position is not too close to the path:
    R3Point lookat_position(lookat_point[0], lookat_point[1], lookat_point[2]);
    R3Vector lookat = lookat_position - viewpoint;
    lookat.Normalize();
    // The fraction should be 30 degrees:
    towards.Normalize();
    for (int vi = 0; vi < 3; ++vi) {
      trajectory->push_back(viewpoint[vi]);
    }
    for (int ti = 0; ti < 3; ++ti) {
      trajectory->push_back(towards[ti]);
    }
  }
  return curve_length;
}


static void
CreatePathInRoomCameras(void)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  RNScalar neardist = 0.01 * scene->BBox().DiagonalRadius();
  RNScalar fardist = 100 * scene->BBox().DiagonalRadius();
  RNScalar aspect = (RNScalar) height / (RNScalar) width;
  RNAngle yfov = atan(aspect * tan(xfov));

  // Create one camera trajectory per door
  for (int i = 0; i < scene->NNodes(); i++) {
    R3SceneNode *room_node = scene->Node(i);
    if (!room_node->Name()) continue;
    if (strncmp(room_node->Name(), "Room#", 5)) continue;
    R3Box room_bbox = room_node->BBox();

    // Compute viewpoint mask
    R2Grid viewpoint_mask;
    if (!ComputeViewpointMask(room_node, viewpoint_mask)) continue;

    // Find the lookat point. TODO(kgenova): Use raycasting for this to ignore occluded faces.
    RNScalar lookat_weight = 0;
    R3Point lookat_position = R3zero_point;
    for (int j = 0; j < room_node->NChildren(); j++) {
      R3SceneNode *child_node = room_node->Child(j);
      RNScalar weight = child_node->NFacets().Max();
      lookat_position += weight * child_node->Centroid();
      lookat_weight += weight;
    }
    if (lookat_weight > 0) lookat_position /= lookat_weight;
    else lookat_position = room_node->Centroid();

    // Mask out a circle of some radius around the lookat point to avoid paths that pass through it.
    float la_z = lookat_position[2];
    float la_x = lookat_position[0];
    float grid_space_r = 15.0;
    float grid_to_world = viewpoint_mask.GridToWorldScaleFactor();
    float r_sq = (grid_to_world * grid_space_r) * (grid_to_world * grid_space_r);
    int num_masked = 0;
    for (int xi = 0; xi < viewpoint_mask.YResolution(); ++xi) {
        for (int zi = 0; zi < viewpoint_mask.XResolution(); ++zi) {
            // Map the points from grid coordinates to world coordinates.
            R2Point world_zx = viewpoint_mask.WorldPosition(zi+0.5, xi+0.5);
            float x_world = world_zx[1];
            float z_world = world_zx[0];
            float dist_sq = (x_world - la_x)*(x_world - la_x) + (z_world - la_z) * (z_world - la_z);
            if (dist_sq < r_sq) {
                viewpoint_mask(zi, xi) = R2_GRID_UNKNOWN_VALUE;
                num_masked++;
            }
        }
    }
    //std::cout << "Masked a total of " << num_masked << " out of " << viewpoint_mask.NEntries() << " grid entries." << std::endl;
    // Find largest connected component_mask
    R2Grid component_mask = viewpoint_mask;
    component_mask.ConnectedComponentSizeFilter(RN_EPSILON);
    RNScalar component_size = component_mask.Maximum();
    component_mask.Threshold(component_size - 0.5, 0, 1.0);
    component_mask.Substitute(0, R2_GRID_UNKNOWN_VALUE);

    // Find path between distant pair of points in largest connected component
    int path_size = 0;
    int random_point_index = FindIndexOfRandomPoint(component_mask);
    if (random_point_index < 0) continue;
    int start_point_index = FindIndexOfFurthestPointAlongPath(component_mask, random_point_index);
    if (start_point_index < 0) continue;
    int *path_indices = new int [ component_mask.NEntries() ];
    int end_point_index = FindIndexOfFurthestPointAlongPath(component_mask, start_point_index, path_indices, &path_size);
    if (end_point_index <= 0) { delete [] path_indices; continue; }

    std::vector<int> useful_ids;
    for (int i = 0; i < path_size; ++i) { 
      if (i % 25 == 0) { 
        useful_ids.push_back(path_indices[i]);
      }
    }
    
    // Trim unnecessary nodes. No longer necessary since we subsample uniformly.
    /*
    useful_ids.push_back(path_indices[0]); // The first index is always necessary.
    for (int i = 1; i < path_size-1; ++i) {
      // Consider a sequence of path steps (prev, i, next).
      // Step i is necessary if the relative motion vector (i - prev) is different
      // from the motion vector (next - i).
      int prev_x, prev_z, i_x, i_z, next_x, next_z;
      component_mask.IndexToIndices(path_indices[i-1], prev_z, prev_x);
      component_mask.IndexToIndices(path_indices[i], i_z, i_x);
      component_mask.IndexToIndices(path_indices[i+1], next_z, next_x);
      int pre_motion_z, pre_motion_x, post_motion_z, post_motion_x;
      pre_motion_z = i_z - prev_z;
      pre_motion_x = i_x - prev_x;
      post_motion_z = next_z - i_z;
      post_motion_x = next_x - i_x;
      bool is_necessary = (pre_motion_z != post_motion_z) || (pre_motion_x != post_motion_x);
      if (is_necessary) {
        useful_ids.push_back(path_indices[i]);
      }
    }
    useful_ids.push_back(path_indices[path_size-1]); // The last index is always necessary.
    std::cout << "There are " << path_size << " total nodes and " << useful_ids.size() << " necessary nodes." << std::endl;*/
       
    // Get the keypoint world coordinates
    std::vector<double> keypoints;
    keypoints.reserve(3*useful_ids.size());
    for (size_t i = 0; i < useful_ids.size(); ++i) {
      int sample_ix, sample_iy;
      int sample_index = useful_ids[i];
      component_mask.IndexToIndices(sample_index, sample_ix, sample_iy);
      R2Point world_zx = component_mask.WorldPosition(sample_ix+0.5, sample_iy+0.5);
      float x = world_zx[1];
      float y = room_bbox.YMin() + eye_height; 
      float z = world_zx[0];

      // Apply some random jitter to each point to better model a human path:
      x += 2.0f*(RNRandomScalar()-0.5f) * eye_height_radius; // abuse of eye height radius
      y += 2.0f*(RNRandomScalar()-0.5f) * eye_height_radius;
      z += 2.0f*(RNRandomScalar()-0.5f) * eye_height_radius; // abuse of eye height radius
      keypoints.push_back(x);
      keypoints.push_back(y);
      keypoints.push_back(z);
    }
     
    float lookat_point[3];
    for (int i = 0; i < 3; ++i) { lookat_point[i] = lookat_position[i]; }
    float lookat_coef = 0.33f; // fractional
    float downward_tilt = 0.18f; // fractional
    float speed = 0.35f; // meters / second. 
    float fps = 30.0f; // frames / second
    std::vector<float> output_trajectory_buffer;
    // The output curve length is in meters
    float curve_length = InterpolateKeypoints(keypoints, lookat_point,
                                              lookat_coef, downward_tilt,
                                              speed, fps, 
                                              &output_trajectory_buffer);
    
    std::vector<R3Camera> trajectory = ConvertTrajectoryToGAPSFormat(
      output_trajectory_buffer, yfov, neardist, fardist);
    std::cout<< "The curve has length " << curve_length << std::endl;
    if (curve_length < 7.0f) { continue; } // Skip short trajectories...
    
    // Ignore the first and last 10% of the trajectory:
    int cutoff_start = trajectory.size() / 10;
    int cutoff_end = (9*trajectory.size()) / 10;
 
    for (int i = cutoff_start; i < cutoff_end; ++i) {
      char name[1024];
      sprintf(name, "%s_%d", room_node->Name(), i);
      Camera* camera = new Camera(trajectory[i], name);
      cameras.Insert(camera);
      camera_count++;
    }

    // Delete temporary memory
    delete [] path_indices;
  }
 
  // Print statistics
  if (print_verbose) {
    printf("Created room cameras ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count++);
    fflush(stdout);
  }
}


static void
CreateInteriorCameras(void)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  RNScalar neardist = 0.01 * scene->BBox().DiagonalRadius();
  RNScalar fardist = 100 * scene->BBox().DiagonalRadius();
  RNScalar aspect = (RNScalar) height / (RNScalar) width;
  RNAngle yfov = atan(aspect * tan(xfov));
  R3Box bbox = scene->BBox();

  // Sample directions
  int nangles = (int) (RN_TWO_PI / angle_sampling + 0.5);
  RNScalar angle_spacing = (nangles > 1) ? RN_TWO_PI / nangles : RN_TWO_PI;
  for (int j = 0; j < nangles; j++) {
    // Choose one camera for each direction 
    R3Camera best_camera;

    // Sample positions 
    for (RNScalar y = bbox.YMin(); y < bbox.YMax(); y += position_sampling) {
      for (RNScalar x = bbox.XMin(); x < bbox.XMax(); x += position_sampling) {
        // Compute position
        R2Point position(x + position_sampling*RNRandomScalar(), y + position_sampling*RNRandomScalar());

        // Compute height
        RNScalar z = bbox.ZMin() + eye_height;
        z += 2.0*(RNRandomScalar()-0.5) * eye_height_radius;
        if (z > bbox.ZMax()) continue;

        // Compute direction
        RNScalar angle = (j+RNRandomScalar()) * angle_spacing;
        R2Vector direction = R2posx_vector;
        direction.Rotate(angle);
        direction.Normalize();

        // Compute camera
        R3Point viewpoint(position[0], position[1], z);
        R3Vector towards(direction.X(), direction.Y(), -0.2);
        towards.Normalize();
        R3Vector right = towards % R3posz_vector;
        right.Normalize();
        R3Vector up = right % towards;
        up.Normalize();
        R3Camera camera(viewpoint, towards, up, xfov, yfov, neardist, fardist);

        // Compute score for camera
        camera.SetValue(SceneCoverageScore(camera, scene));
        if (camera.Value() == 0) continue;
        if (camera.Value() < min_score) continue;

        // Remember best camera
        if (camera.Value() > best_camera.Value()) {
          best_camera = camera;
        }
      }
    }

    // Insert best camera for direction
    if (best_camera.Value() > 0) {
      if (print_debug) printf("INTERIOR %d : %g\n", j, best_camera.Value());
      char name[1024];
      sprintf(name, "C_%d", j);
      Camera *camera = new Camera(best_camera, name);
      cameras.Insert(camera);
      camera_count++;
    }
  }
        
  // Print statistics
  if (print_verbose) {
    printf("Created interior cameras ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count++);
    fflush(stdout);
  }
}



static void
CreateWorldInHandCameras(void)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  R3Point centroid = scene->Centroid();
  RNScalar radius = scene->BBox().DiagonalRadius();
  RNScalar neardist = 0.01 * radius;
  RNScalar fardist = 100 * radius;
  RNScalar aspect = (RNScalar) height / (RNScalar) width;
  RNAngle yfov = atan(aspect * tan(xfov));
  RNLength distance = 2.5 * radius;

  // Determine number of cameras
  int ncameras = 0;
  if (position_sampling > 0) {
    RNArea area_of_viewpoint_sphere = 4.0 * RN_PI * distance * distance;
    RNScalar area_per_camera = 4.0 * position_sampling * position_sampling;
    int position_ncameras = (int) (area_of_viewpoint_sphere / area_per_camera + 0.5);
    if (position_ncameras > ncameras) ncameras = position_ncameras;
  }
  if (angle_sampling > 0) {
    RNArea area_of_viewpoint_sphere = 4.0 * RN_PI * distance * distance;
    RNScalar arc_length = (distance * angle_sampling);
    RNScalar area_per_camera = arc_length * arc_length;
    int angle_ncameras = (int) (area_of_viewpoint_sphere / area_per_camera + 0.5);
    if (angle_ncameras > ncameras) ncameras = angle_ncameras;
  }
  if (ncameras == 0) ncameras = 1024;
  
  // Create cameras at random directions from viewpoint sphere looking at centroid
  for (int i = 0; i < ncameras; i++) {
    // Compute view directions
    R3Vector towards = R3RandomDirection();
    towards.Normalize();
    R3Vector right = towards % R3posy_vector;
    if (RNIsZero(right.Length())) continue;
    right.Normalize();
    R3Vector up = right % towards;
    if (RNIsZero(up.Length())) continue;
    up.Normalize();

    // Compute eyepoint
    RNScalar d = distance + (2.0*RNRandomScalar()-1.0) * position_sampling;
    R3Point viewpoint = centroid - d * towards;

    // Compute name
    char name[1024];
    sprintf(name, "WORLDINHAND#%d", i);

    // Create camera
    R3Camera c(viewpoint, towards, up, xfov, yfov, neardist, fardist);
    Camera *camera = new Camera(c, name);
    cameras.Insert(camera);
    camera_count++;
  }

  // Print statistics
  if (print_verbose) {
    printf("Created world in hand cameras ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count++);
    fflush(stdout);
  }
}



////////////////////////////////////////////////////////////////////////
// Camera interpolation functions
////////////////////////////////////////////////////////////////////////

static int
InterpolateCameraTrajectory(RNLength trajectory_step = 0.1)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Set some camera parameters based on first camera
  RNLength xf = cameras.Head()->XFOV();
  RNLength yf = cameras.Head()->YFOV();
  RNLength neardist = cameras.Head()->Near();
  RNLength fardist = cameras.Head()->Far();
  
  // Create spline data
  int nkeypoints = cameras.NEntries();
  R3Point *viewpoint_keypoints =  new R3Point [ nkeypoints ];
  R3Point *towards_keypoints =  new R3Point [ nkeypoints ];
  R3Point *up_keypoints =  new R3Point [ nkeypoints ];
  RNScalar *parameters = new RNScalar [ nkeypoints ];
  for (int i = 0; i < cameras.NEntries(); i++) {
    R3Camera *camera = cameras.Kth(i);
    viewpoint_keypoints[i] = camera->Origin();
    towards_keypoints[i] = camera->Towards().Point();
    up_keypoints[i] = camera->Up().Point();
    if (i == 0) parameters[i] = 0;
    else parameters[i] = parameters[i-1] + R3Distance(viewpoint_keypoints[i], viewpoint_keypoints[i-1]) + R3InteriorAngle(towards_keypoints[i].Vector(), towards_keypoints[i-1].Vector());
  }

  // Create splines
  R3CatmullRomSpline viewpoint_spline(viewpoint_keypoints, parameters, nkeypoints);
  R3CatmullRomSpline towards_spline(towards_keypoints, parameters, nkeypoints);
  R3CatmullRomSpline up_spline(up_keypoints, parameters, nkeypoints);

  // Delete cameras
  for (int i = 0; i < cameras.NEntries(); i++) delete cameras[i];
  cameras.Empty();
  
  // Resample splines
  for (RNScalar u = viewpoint_spline.StartParameter(); u <= viewpoint_spline.EndParameter(); u += trajectory_step) {
    R3Point viewpoint = viewpoint_spline.PointPosition(u);
    R3Point towards = towards_spline.PointPosition(u);
    R3Point up = up_spline.PointPosition(u);
    Camera *camera = new Camera(viewpoint, towards.Vector(), up.Vector(), xf, yf, neardist, fardist);
    char name[1024];
    sprintf(name, "T%f", u);
    camera->name = strdup(name);
    cameras.Insert(camera);
  }

  // Delete spline data
  delete [] viewpoint_keypoints;
  delete [] towards_keypoints;
  delete [] up_keypoints;
  delete [] parameters;

  // Print statistics
  if (print_verbose) {
    printf("Interpolated camera trajectory ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", cameras.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



////////////////////////////////////////////////////////////////////////
// Camera processing functions
////////////////////////////////////////////////////////////////////////

static int
SortCameras(void)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Sort the cameras
  cameras.Sort(R3CompareCameras);

  // Print statistics
  if (print_verbose) {
    printf("Sorted cameras ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", cameras.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



////////////////////////////////////////////////////////////////////////
// Create and write functions
////////////////////////////////////////////////////////////////////////

static void
CreateAndWriteCameras(void)
{
  // Create cameras
  if (create_object_cameras) CreateObjectCameras();
  if (create_interior_cameras) CreateInteriorCameras();
  if (create_world_in_hand_cameras) CreateWorldInHandCameras();

  // Create specialized cameras (for SUNCG)
  if (create_room_cameras) CreateRoomCameras();
  if (create_path_in_room_cameras) CreatePathInRoomCameras();

  // Create trajectory from cameras
  if (interpolate_camera_trajectory) {
    if (!InterpolateCameraTrajectory(interpolation_step)) exit(-1);
  }
  else {
    SortCameras();
  }

  // Write cameras
  WriteCameras();

  // Exit program
  exit(0);
}



static int
CreateAndWriteCamerasWithGlut(void)
{
#ifdef USE_GLUT
  // Open window
  int argc = 1;
  char *argv[1];
  argv[0] = strdup("scn2cam");
  glutInit(&argc, argv);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(width, height);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH); 
  glutCreateWindow("Scene Camera Creation");

  // Initialize GLUT callback functions 
  glutDisplayFunc(CreateAndWriteCameras);

  // Run main loop  -- never returns 
  glutMainLoop();

  // Return success -- actually never gets here
  return 1;
#else
  // Not supported
  RNAbort("Program was not compiled with glut.  Recompile with make.\n");
  return 0;
#endif
}



static int
CreateAndWriteCamerasWithMesa(void)
{
#ifdef USE_MESA
  // Create mesa context
  OSMesaContext ctx = OSMesaCreateContextExt(OSMESA_RGBA, 32, 0, 0, NULL);
  if (!ctx) {
    fprintf(stderr, "Unable to create mesa context\n");
    return 0;
  }

  // Create frame buffer
  void *frame_buffer = malloc(width * height * 4 * sizeof(GLubyte) );
  if (!frame_buffer) {
    fprintf(stderr, "Unable to allocate mesa frame buffer\n");
    return 0;
  }

  // Assign mesa context
  if (!OSMesaMakeCurrent(ctx, frame_buffer, GL_UNSIGNED_BYTE, width, height)) {
    fprintf(stderr, "Unable to make mesa context current\n");
    return 0;
  }

  // Create cameras
  CreateAndWriteCameras();

  // Delete mesa context
  OSMesaDestroyContext(ctx);

  // Delete frame buffer
  free(frame_buffer);

  // Return success
  return 1;
#else
  // Not supported
  RNAbort("Program was not compiled with mesa.  Recompile with make mesa.\n");
  return 0;
#endif
}



////////////////////////////////////////////////////////////////////////
// Program argument parsing
////////////////////////////////////////////////////////////////////////

static int 
ParseArgs(int argc, char **argv)
{
  // Initialize variables to track whether to assign defaults
  int create_cameras = 0;
  int output = 0;
  
  // Parse arguments
  argc--; argv++;
  while (argc > 0) {
    if ((*argv)[0] == '-') {
      if (!strcmp(*argv, "-v")) print_verbose = 1;
      else if (!strcmp(*argv, "-debug")) print_debug = 1;
      else if (!strcmp(*argv, "-glut")) { mesa = 0; glut = 1; }
      else if (!strcmp(*argv, "-mesa")) { mesa = 1; glut = 0; }
      else if (!strcmp(*argv, "-raycast")) { mesa = 0; glut = 0; }
      else if (!strcmp(*argv, "-categories")) { argc--; argv++; input_categories_filename = *argv; }
      else if (!strcmp(*argv, "-input_cameras")) { argc--; argv++; input_cameras_filename = *argv; }
      else if (!strcmp(*argv, "-output_camera_extrinsics")) { argc--; argv++; output_camera_extrinsics_filename = *argv; output = 1; }
      else if (!strcmp(*argv, "-output_camera_intrinsics")) { argc--; argv++; output_camera_intrinsics_filename = *argv; output = 1; }
      else if (!strcmp(*argv, "-output_camera_names")) { argc--; argv++; output_camera_names_filename = *argv; output = 1; }
      else if (!strcmp(*argv, "-output_nodes")) { argc--; argv++; output_nodes_filename = *argv; output = 1; }
      else if (!strcmp(*argv, "-interpolate_camera_trajectory")) { interpolate_camera_trajectory = 1; }
      else if (!strcmp(*argv, "-width")) { argc--; argv++; width = atoi(*argv); }
      else if (!strcmp(*argv, "-height")) { argc--; argv++; height = atoi(*argv); }
      else if (!strcmp(*argv, "-xfov")) { argc--; argv++; xfov = atof(*argv); }
      else if (!strcmp(*argv, "-eye_height")) { argc--; argv++; eye_height = atof(*argv); }
      else if (!strcmp(*argv, "-eye_height_radius")) { argc--; argv++; eye_height_radius = atof(*argv); }
      else if (!strcmp(*argv, "-min_distance_from_obstacle")) { argc--; argv++; min_distance_from_obstacle = atof(*argv); }
      else if (!strcmp(*argv, "-min_visible_objects")) { argc--; argv++; min_visible_objects = atoi(*argv); }
      else if (!strcmp(*argv, "-min_score")) { argc--; argv++; min_score = atof(*argv); }
      else if (!strcmp(*argv, "-scene_scoring_method")) { argc--; argv++; scene_scoring_method = atoi(*argv); }
      else if (!strcmp(*argv, "-object_scoring_method")) { argc--; argv++; object_scoring_method = atoi(*argv); }
      else if (!strcmp(*argv, "-position_sampling")) { argc--; argv++; position_sampling = atof(*argv); }
      else if (!strcmp(*argv, "-angle_sampling")) { argc--; argv++; angle_sampling = atof(*argv); }
      else if (!strcmp(*argv, "-interpolation_step")) { argc--; argv++; interpolation_step = atof(*argv); }
      else if (!strcmp(*argv, "-create_object_cameras") || !strcmp(*argv, "-create_leaf_node_cameras")) {
        create_cameras = create_object_cameras = 1;
        angle_sampling = RN_PI / 6.0;
      }
      else if (!strcmp(*argv, "-create_interior_cameras")) {
        create_cameras = create_interior_cameras = 1;
        angle_sampling = RN_PI / 2.0;
      }
      else if (!strcmp(*argv, "-create_room_cameras")) {
        create_cameras = create_room_cameras = 1;
        angle_sampling = RN_PI / 2.0;
      }
      else if (!strcmp(*argv, "-create_path_in_room_cameras")) {
        create_cameras = create_path_in_room_cameras = 1;
      }
      else if (!strcmp(*argv, "-create_world_in_hand_cameras")) {
        create_cameras = create_world_in_hand_cameras = 1;
      }
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!input_scene_filename) input_scene_filename = *argv;
      else if (!output_cameras_filename) { output_cameras_filename = *argv; output = 1; }
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Set default camera options
  if (!input_cameras_filename && !create_cameras) {
    create_room_cameras = 1;
  }

  // Check filenames
  if (!input_scene_filename || !output) {
    fprintf(stderr, "Usage: scn2cam inputscenefile outputcamerafile\n");
    return 0;
  }

  // Return OK status 
  return 1;
}



////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  // Parse program arguments
  if (!ParseArgs(argc, argv)) exit(-1);

  // Read scene
  if (!ReadScene(input_scene_filename)) exit(-1);

  // Read cameras
  if (input_cameras_filename) {
    if (!ReadCameras(input_cameras_filename)) exit(-1);
  }

  // Read categories
  if (input_categories_filename) {
    if (!ReadCategories(input_categories_filename)) exit(-1);
  }

  // Create and write new cameras 
  if (mesa) CreateAndWriteCamerasWithMesa();
  else if (glut) CreateAndWriteCamerasWithGlut();
  else CreateAndWriteCameras();

  // Return success 
  return 0;
}




