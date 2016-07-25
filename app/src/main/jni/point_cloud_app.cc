/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tango-gl/conversions.h>
#include <tango_support_api.h>

#include "tango-point-cloud/point_cloud_app.h"

namespace {
  const int kVersionStringLength = 128;

  void onFrameAvailableRouter(void* context, TangoCameraId, const TangoImageBuffer* buffer) {
    tango_point_cloud::PointCloudApp* app =  static_cast<tango_point_cloud::PointCloudApp*>(context);
    app->onFrameAvailable(buffer);
  }

  void onPointCloudAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
    tango_point_cloud::PointCloudApp* app = static_cast<tango_point_cloud::PointCloudApp*>(context);
    app->onPointCloudAvailable(xyz_ij);
  }
}

namespace tango_point_cloud {

  void PointCloudApp::onFrameAvailable(const TangoImageBuffer* buffer) {
    TangoSupport_updateImageBuffer(yuv_manager_, buffer);
    //LOGE("buffer stride %i", buffer->stride);
  }

  void PointCloudApp::onPointCloudAvailable(const TangoXYZij* xyz_ij) {
    TangoSupport_updatePointCloud(point_cloud_manager_, xyz_ij);
  }

  PointCloudApp::PointCloudApp() {}

  PointCloudApp::~PointCloudApp() {

    if (tango_config_ != nullptr) {
      TangoConfig_free(tango_config_);
    }

    if (point_cloud_manager_ != nullptr) {
      TangoSupport_freePointCloudManager(point_cloud_manager_);
      point_cloud_manager_ = nullptr;
    }

  }

  bool PointCloudApp::CheckTangoVersion(JNIEnv* env, jobject activity, int min_tango_version) {
    int version;
    TangoErrorType err = TangoSupport_GetTangoVersion(env, activity, &version);
    return err == TANGO_SUCCESS && version >= min_tango_version;
  }

  bool PointCloudApp::OnTangoServiceConnected(JNIEnv* env, jobject binder) {
    TangoErrorType ret = TangoService_setBinder(env, binder);
    if (ret != TANGO_SUCCESS) {
      LOGE("PointCloudApp: Failed to set Binder Tango service with error code: %d", ret);
      return false;
    }
    return true;
  }

  int PointCloudApp::TangoSetupConfig() {

    tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);

    if (tango_config_ == nullptr) {
      LOGE("PointCloudApp: Failed to get default config form");
      return TANGO_ERROR;
    }

    int ret = TangoConfig_setBool(tango_config_, "config_enable_auto_recovery", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("PointCloudApp: config_enable_auto_recovery() failed with error code: %d", ret);
      return ret;
    }

    ret = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("PointCloudApp: config_enable_depth() failed with error code: %d", ret);
      return ret;
    }

    if (point_cloud_manager_ == nullptr) {
      int32_t max_point_cloud_elements;
      ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements", &max_point_cloud_elements);
      if (ret != TANGO_SUCCESS) {
        LOGE("Failed to query maximum number of point cloud elements.");
        return false;
      }

      ret = TangoSupport_createPointCloudManager(max_point_cloud_elements, &point_cloud_manager_);
      if (ret != TANGO_SUCCESS) {
        LOGE("Failed to create point cloud manager");
        return false;
      }
    }

    ret = TangoSupport_createImageBufferManager(TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP, 1280, 720, &yuv_manager_);
    if (ret != TANGO_SUCCESS) {
      LOGE("Failed to create image buffer manager");
      return false;
    }

    // Set Tango3DR_ConfigH and Tango3DR_Context for 3D reconstruction
    config_ = Tango3DR_Config_create(TANGO_3DR_CONFIG_CONTEXT);
    Tango3DR_Config_setDouble(config_, "resolution", 0.01);
    Tango3DR_Config_setDouble(config_, "min_depth", 0.25);
    Tango3DR_Config_setDouble(config_, "max_depth", 2.0);
    Tango3DR_Config_setBool(config_, "generate_color", true);
    context_ = Tango3DR_create(config_);

    Tango3DR_Config_destroy(config_);

    gridindexarray_ = new Tango3DR_GridIndexArray();
    mesh_ = new Tango3DR_Mesh();

    return ret;
  }

  int PointCloudApp::TangoConnectCallbacks() {

    int ret = TangoService_connectOnXYZijAvailable(onPointCloudAvailableRouter);
    if (ret != TANGO_SUCCESS) {
      LOGE("PointCloudApp: Failed to connect to point cloud callback with errorcode: %d", ret);
      return ret;
    }

    ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, onFrameAvailableRouter);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to connect to on frame available callback with error"
           "code: %d", ret);
      return ret;
    }

    return ret;
  }

  bool PointCloudApp::TangoConnect() {
    TangoErrorType err = TangoService_connect(this, tango_config_);
    if (err != TANGO_SUCCESS) {
      LOGE(
          "PointCloudApp: Failed to connect to the Tango service with"
          "error code: %d",
          err);
      return false;
    }

    TangoSupport_initialize(TangoService_getPoseAtTime);
    return true;
  }

  void PointCloudApp::TangoDisconnect() {
    TangoConfig_free(tango_config_);
    tango_config_ = nullptr;
    TangoService_disconnect();
  }

  void PointCloudApp::InitializeGLContent() {
    main_scene_.InitGLContent();

    TangoErrorType err = TangoService_getCameraIntrinsics(
          TANGO_CAMERA_COLOR, &color_camera_intrinsics_);
      if (err != TANGO_SUCCESS) {
        LOGE( "Failed to get the intrinsics for the colorcamera.");
      }

    // Set Tango3DR_CameraCalibration
    color_camera_.height = color_camera_intrinsics_.height;
    color_camera_.width = color_camera_intrinsics_.width;

    color_camera_.distortion[0] = color_camera_intrinsics_.distortion[0];
    color_camera_.distortion[1] = color_camera_intrinsics_.distortion[1];
    color_camera_.distortion[2] = color_camera_intrinsics_.distortion[2];
    color_camera_.distortion[3] = color_camera_intrinsics_.distortion[3];
    color_camera_.distortion[4] = color_camera_intrinsics_.distortion[4];

    color_camera_.fx = color_camera_intrinsics_.fx;
    color_camera_.fy = color_camera_intrinsics_.fy;
    color_camera_.cx = color_camera_intrinsics_.cx;
    color_camera_.cy = color_camera_intrinsics_.cy;

    color_camera_.calibration_type = (Tango3DR_TangoCalibrationType)color_camera_intrinsics_.calibration_type;
  }

  void PointCloudApp::SetViewPort(int width, int height) {
    main_scene_.SetupViewPort(width, height);
  }

  void PointCloudApp::Render() {

    UpdateCurrentPointData();

    // Copy TangoXYZij to Tango3DR_PointCloud
    Tango3DR_PointCloud pointcloud;
    pointcloud.num_points = front_cloud_->xyz_count;
    pointcloud.points = new Tango3DR_Vector4[pointcloud.num_points];
    for (int i = 0; i < pointcloud.num_points; ++i) {
      pointcloud.points[i][0] = front_cloud_->xyz[i][0];
      pointcloud.points[i][1] = front_cloud_->xyz[i][1];
      pointcloud.points[i][2] = front_cloud_->xyz[i][2];
      // last is confidence
      pointcloud.points[i][3] = 1;
    }
    pointcloud.timestamp = front_cloud_->timestamp;

    Tango3DR_ImageBuffer color_image;
    TangoImageBuffer* image_buffer;

    int ret = TangoSupport_getLatestImageBuffer(yuv_manager_, &image_buffer);

    // Copy TangoImageBuffer to Tango3DR_ImageBuffer
    if (ret != TANGO_SUCCESS) {
      LOGE("NO IMAGE AVAILABLE!");
    } else {
      color_image.width = image_buffer->width;
      color_image.height = image_buffer->height;
      color_image.stride = 1280;
      color_image.timestamp = image_buffer->timestamp;
      color_image.format = (Tango3DR_ImageFormatType)image_buffer->format;
      color_image.data = image_buffer->data;
    }

    //LOGE("stride %i", image_buffer->stride);

    TangoPoseData pointcloud_pose_tmp, imagebuffer_pose_tmp, current_pose;
    Tango3DR_Pose pointcloud_pose, imagebuffer_pose;

    TangoCoordinateFramePair frame_pair;
    frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;

    TangoService_getPoseAtTime(pointcloud.timestamp, frame_pair, &current_pose);
    TangoService_getPoseAtTime(pointcloud.timestamp, frame_pair, &pointcloud_pose_tmp);
    TangoService_getPoseAtTime(color_image.timestamp, frame_pair, &imagebuffer_pose_tmp);

    // Copy TangoPoseData to Tango3DR_Pose
    if(pointcloud_pose_tmp.status_code != TANGO_POSE_VALID) {
        LOGE("pointcloud_pose_tmp invalid");
    } else {
        pointcloud_pose.orientation[0] = pointcloud_pose_tmp.orientation[0];
        pointcloud_pose.orientation[1] = pointcloud_pose_tmp.orientation[1];
        pointcloud_pose.orientation[2] = pointcloud_pose_tmp.orientation[2];
        pointcloud_pose.orientation[3] = pointcloud_pose_tmp.orientation[3];

        pointcloud_pose.translation[0] = pointcloud_pose_tmp.translation[0];
        pointcloud_pose.translation[1] = pointcloud_pose_tmp.translation[1];
        pointcloud_pose.translation[2] = pointcloud_pose_tmp.translation[2];
    }

    if(imagebuffer_pose_tmp.status_code != TANGO_POSE_VALID) {
        LOGE("imagebuffer_pose_tmp invalid");
    } else {
        imagebuffer_pose.orientation[0] = imagebuffer_pose_tmp.orientation[0];
        imagebuffer_pose.orientation[1] = imagebuffer_pose_tmp.orientation[1];
        imagebuffer_pose.orientation[2] = imagebuffer_pose_tmp.orientation[2];
        imagebuffer_pose.orientation[3] = imagebuffer_pose_tmp.orientation[3];

        imagebuffer_pose.translation[0] = imagebuffer_pose_tmp.translation[0];
        imagebuffer_pose.translation[1] = imagebuffer_pose_tmp.translation[1];
        imagebuffer_pose.translation[2] = imagebuffer_pose_tmp.translation[2];
    }

    // Passing with nullptr and the current mesh will be rendered
    /*Tango3DR_Status status = Tango3DR_update(context_, &pointcloud, &pointcloud_pose,
                                             nullptr, nullptr, nullptr, &gridindexarray_);*/

    // Passing with Tango3DR_ImageBuffer a TANGO_3DR_INVALID error will be returned
    Tango3DR_Status status = Tango3DR_update(context_, &pointcloud, &pointcloud_pose,
                                             &color_image, &imagebuffer_pose, &color_camera_,
                                             &gridindexarray_);

    if (status == TANGO_3DR_ERROR) {
      LOGE("UPDATE STATUS some sort of hard error occurred");
    }
    if (status == TANGO_3DR_INSUFFICIENT_SPACE) {
      LOGE("UPDATE STATUS not enough space in a provided buffer");
    }
    if (status == TANGO_3DR_INVALID) {
      LOGE("UPDATE STATUS input argument is invalid");
    }
    if (status == TANGO_3DR_SUCCESS) {
      LOGE("UPDATE STATUS success");

      status = Tango3DR_extractMesh(context_, gridindexarray_, &mesh_);

      if (status == TANGO_3DR_ERROR) {
          LOGE("EXTRACT STATUS some sort of hard error occurred");
      }

      if (status == TANGO_3DR_INSUFFICIENT_SPACE) {
          LOGE("EXTRACT STATUS not enough space in a provided buffer");
      }

      if (status == TANGO_3DR_INVALID) {
          LOGE("EXTRACT STATUS input argument is invalid");
      }

      if (status == TANGO_3DR_SUCCESS) {
          LOGE("EXTRACT STATUS success");

          size_t point_cloud_size = mesh_->num_vertices * 3;
          std::vector<GLfloat> vertices_tmp;
          std::vector<float> vertices;
          std::vector<uint8_t> colors;
          if (mesh_->num_vertices > 0) {
              for (int i=0; i<mesh_->num_vertices; i++) {
                  vertices.push_back(mesh_->vertices[i][0]);
                  vertices.push_back(mesh_->vertices[i][1]);
                  vertices.push_back(mesh_->vertices[i][2]);
                  colors.push_back(mesh_->colors[i][0]);
                  colors.push_back(mesh_->colors[i][1]);
                  colors.push_back(mesh_->colors[i][2]);
              }
              main_scene_.Render(GetMatrixFromPose(current_pose), vertices, colors);
          }
      }
    }
  }

  void PointCloudApp::DeleteResources() {
    main_scene_.DeleteResources();
  }

  int PointCloudApp::GetPointCloudVerticesCount() {
    if (front_cloud_ != nullptr) {
      return front_cloud_->xyz_count;
    }
    return 0;
  }

  float PointCloudApp::GetAverageZ() {
    return point_cloud_average_depth_;
  }

  void PointCloudApp::SetCameraType(
      tango_gl::GestureCamera::CameraType camera_type) {
    main_scene_.SetCameraType(camera_type);
  }

  void PointCloudApp::OnTouchEvent(int touch_count,
                                   tango_gl::GestureCamera::TouchEvent event,
                                   float x0, float y0, float x1, float y1) {
    main_scene_.OnTouchEvent(touch_count, event, x0, y0, x1, y1);
  }

  void PointCloudApp::UpdateCurrentPointData() {
    TangoSupport_getLatestPointCloud(point_cloud_manager_, &front_cloud_);
  }

  void PointCloudApp::SetScreenRotation(int screen_rotation) {
    screen_rotation_ = screen_rotation;
  }

  glm::mat4 PointCloudApp::GetMatrixFromPose(const TangoPoseData& pose) {
    glm::vec3 translation = glm::vec3(pose.translation[0], pose.translation[1], pose.translation[2]);
    glm::quat rotation = glm::quat(pose.orientation[3], pose.orientation[0],
                                   pose.orientation[1], pose.orientation[2]);
    glm::mat4 matrix = glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);
    return matrix;
  }
}  // namespace tango_point_cloud
