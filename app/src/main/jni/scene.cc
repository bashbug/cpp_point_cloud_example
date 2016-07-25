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

#include "tango-point-cloud/scene.h"

namespace {
// We want to represent the device properly with respect to the ground so we'll
// add an offset in z to our origin. We'll set this offset to 1.3 meters based
// on the average height of a human standing with a Tango device. This allows us
// to place a grid roughly on the ground for most users.
const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Color of the motion tracking trajectory.
const tango_gl::Color kTraceColor(0.22f, 0.28f, 0.67f);

// Color of the ground grid.
const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// Frustum scale.
const glm::vec3 kFrustumScale = glm::vec3(0.4f, 0.3f, 0.5f);
}  // namespace

namespace tango_point_cloud {

Scene::Scene() {}

Scene::~Scene() {}

void Scene::InitGLContent() {
  gesture_camera_ = new tango_gl::GestureCamera();
  axis_ = new tango_gl::Axis();
  frustum_ = new tango_gl::Frustum();
  trace_ = new tango_gl::Trace();
  grid_ = new tango_gl::Grid();
  point_cloud_ = new PointCloudDrawable();

  //mesh_ = new tango_gl::Mesh(GL_POINTS);
  //mesh_->SetShader();
  //mesh_->SetColor(1.0, 0.0, 0.0);


  trace_->SetColor(kTraceColor);
  grid_->SetColor(kGridColor);
  grid_->SetPosition(-kHeightOffset);
  gesture_camera_->SetCameraType(
      tango_gl::GestureCamera::CameraType::kThirdPerson);
}

void Scene::DeleteResources() {
  if (gesture_camera_ != nullptr) {
    delete gesture_camera_;
    gesture_camera_ = nullptr;
  }

  if (frustum_ != nullptr) {
    delete frustum_;
    frustum_ = nullptr;
  }

  if (trace_ != nullptr) {
    delete trace_;
    trace_ = nullptr;
  }

  if (point_cloud_ != nullptr) {
    delete point_cloud_;
    point_cloud_ = nullptr;
  }

  if (axis_ != nullptr) {
    delete axis_;
    axis_ = nullptr;
  }

  if (grid_ != nullptr) {
    delete grid_;
    grid_ = nullptr;
  }
}

void Scene::SetupViewPort(int w, int h) {
  if (h == 0) {
    LOGE("Setup graphic height not valid");
  }
  gesture_camera_->SetAspectRatio(static_cast<float>(w) /
                                  static_cast<float>(h));
  glViewport(0, 0, w, h);
}

void Scene::Render(const glm::mat4& cur_pose_transformation,
                   const std::vector<float>& point_cloud_vertices,
                   const std::vector<uint8_t>& point_cloud_colors) {


  glm::mat4 curr_T_opengl = tango_gl::conversions::opengl_world_T_tango_world()*cur_pose_transformation;
  glm::mat4 mm = curr_T_opengl * glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 1.0f, 0.0f,
                                            0.0f, -1.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f, 1.0f);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  glm::vec3 position =
      glm::vec3(curr_T_opengl[3][0], curr_T_opengl[3][1],
                curr_T_opengl[3][2]);

  if (gesture_camera_->GetCameraType() ==
      tango_gl::GestureCamera::CameraType::kFirstPerson) {
    // In first person mode, we directly control camera's motion.
    gesture_camera_->SetTransformationMatrix(curr_T_opengl);
  } else {
    // In third person or top down more, we follow the camera movement.
    gesture_camera_->SetAnchorPosition(position);

    frustum_->SetTransformationMatrix(curr_T_opengl);
    // Set the frustum scale to 4:3, this doesn't necessarily match the physical
    // camera's aspect ratio, this is just for visualization purposes.
    frustum_->SetScale(kFrustumScale);
    frustum_->Render(gesture_camera_->GetProjectionMatrix(),
                     gesture_camera_->GetViewMatrix());

    axis_->SetTransformationMatrix(curr_T_opengl);
    axis_->Render(gesture_camera_->GetProjectionMatrix(),
                  gesture_camera_->GetViewMatrix());
  }

  trace_->UpdateVertexArray(position);
  trace_->Render(gesture_camera_->GetProjectionMatrix(),
                 gesture_camera_->GetViewMatrix());

  grid_->Render(gesture_camera_->GetProjectionMatrix(),
                gesture_camera_->GetViewMatrix());

  point_cloud_->Render(gesture_camera_->GetProjectionMatrix(),
                       gesture_camera_->GetViewMatrix(),
                       cur_pose_transformation,
                       point_cloud_vertices,
                       point_cloud_colors);
}

void Scene::SetCameraType(tango_gl::GestureCamera::CameraType camera_type) {
  gesture_camera_->SetCameraType(camera_type);
}

void Scene::OnTouchEvent(int touch_count,
                         tango_gl::GestureCamera::TouchEvent event, float x0,
                         float y0, float x1, float y1) {
  gesture_camera_->OnTouchEvent(touch_count, event, x0, y0, x1, y1);
}

glm::mat4 Scene::GetExtrinsicsAppliedOpenGLWorldFrame(const glm::mat4 pose_matrix) {
    return tango_gl::conversions::opengl_world_T_tango_world() * pose_matrix;
}


}  // namespace tango_point_cloud
