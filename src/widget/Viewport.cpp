#include "Viewport.h"

#include "data/misc.h"

#include <glow/ScopedBinder.h>
#include <glow/glutil.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include "rv/Stopwatch.h"

using namespace glow;
using namespace rv;

Viewport::Viewport(QWidget* parent, Qt::WindowFlags f)
    : QGLWidget(parent, 0, f),
      contextInitialized_(initContext()),
      mAxis(XYZ),
      mMode(NONE),
      mFlags(FLAG_OVERWRITE),
      buttonPressed(false),
      texLabelColors_(256, 1, TextureFormat::RGB) {
  connect(&timer_, &QTimer::timeout, [this]() { this->updateGL(); });

  //  setMouseTracking(true);

  drawingOption_["coordinate axis"] = true;

  conversion_ = RoSe2GL::matrix;

  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufRemissions_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);

  initPrograms();
  initVertexBuffers();

  drawingOption_["remission"] = true;
  drawingOption_["color"] = false;
  drawingOption_["single scan"] = false;
  drawingOption_["show all points"] = false;
  drawingOption_["show points"] = true;
  drawingOption_["show train"] = true;
  drawingOption_["show points"] = true;
  drawingOption_["show voxels"] = false;

  texLabelColors_.setMinifyingOperation(TexRectMinOp::NEAREST);
  texLabelColors_.setMagnifyingOperation(TexRectMagOp::NEAREST);

  setAutoFillBackground(false);

  glow::_CheckGlError(__FILE__, __LINE__);
}

Viewport::~Viewport() {}

void Viewport::initPrograms() {
  prgDrawPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_points.vert"));
  prgDrawPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPoints_.link();

  prgDrawPose_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/empty.vert"));
  prgDrawPose_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_pose.geom"));
  prgDrawPose_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/passthrough.frag"));
  prgDrawPose_.link();

  prgDrawVoxels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shaders/draw_voxels.vert"));
  prgDrawVoxels_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shaders/draw_voxels.geom"));
  prgDrawVoxels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shaders/blinnphong.frag"));
  prgDrawVoxels_.link();
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_points_.setVertexAttribute(1, bufRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_points_.setVertexAttribute(2, bufLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(3, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);

  vao_prior_voxels_.setVertexAttribute(0, bufPriorVoxels_, 3, AttributeType::FLOAT, false, sizeof(LabeledVoxel),
                                       reinterpret_cast<GLvoid*>(0));
  vao_prior_voxels_.setVertexAttribute(1, bufPriorVoxels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(LabeledVoxel),
                                       reinterpret_cast<GLvoid*>(sizeof(glow::vec3)));

  vao_past_voxels_.setVertexAttribute(0, bufPastVoxels_, 3, AttributeType::FLOAT, false, sizeof(LabeledVoxel),
                                      reinterpret_cast<GLvoid*>(0));
  vao_past_voxels_.setVertexAttribute(1, bufPastVoxels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(LabeledVoxel),
                                      reinterpret_cast<GLvoid*>(sizeof(glow::vec3)));
}

void Viewport::setMaximumScans(uint32_t numScans) {
  maxScans_ = numScans;

  uint32_t max_size = maxScans_ * maxPointsPerScan_;

  bufPoses_.resize(maxScans_);
  bufPoints_.resize(max_size);
  bufRemissions_.resize(max_size);
  bufVisible_.resize(max_size);
  bufLabels_.resize(max_size);
}

void Viewport::setVoxels(const std::vector<LabeledVoxel>& priorVoxels, const std::vector<LabeledVoxel>& pastVoxels) {
  bufPriorVoxels_.resize(priorVoxels.size());
  bufPastVoxels_.resize(pastVoxels.size());

  bufPriorVoxels_.assign(priorVoxels);
  bufPastVoxels_.assign(pastVoxels);

  updateGL();
}

void Viewport::setVoxelGridProperties(float voxelSize, const Eigen::Vector4f& offset) {
  prgDrawVoxels_.setUniform(GlUniform<float>("voxelSize", voxelSize));
  prgDrawVoxels_.setUniform(GlUniform<Eigen::Vector4f>("voxelOffset", offset));

  updateGL();
}

void Viewport::setPoints(const std::vector<PointcloudPtr>& priorPoints, std::vector<LabelsPtr>& priorLabels,
                         const std::vector<PointcloudPtr>& pastPoints, std::vector<LabelsPtr>& pastLabels) {
  std::cout << "Setting points..." << std::flush;

  glow::_CheckGlError(__FILE__, __LINE__);

  // FIXME: improve usage of resources:
  //   Use transform feedback to get points inside the tile.

  priorPoints_ = priorPoints;
  priorLabels_ = priorLabels;
  priorIndexes_.clear();

  pastPoints_ = pastPoints;
  pastLabels_ = pastLabels;
  pastIndexes_.clear();

  glow::_CheckGlError(__FILE__, __LINE__);

  {
    // first remove entries using a set_difference
    std::vector<Laserscan*> before;
    std::vector<Laserscan*> after;
    for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) before.push_back(it->first);
    for (auto it = priorPoints.begin(); it != priorPoints.end(); ++it) after.push_back(it->get());
    for (auto it = pastPoints.begin(); it != pastPoints.end(); ++it) after.push_back(it->get());

    std::sort(before.begin(), before.end());
    std::sort(after.begin(), after.end());

    std::vector<Laserscan*> needsDelete(before.size());
    std::vector<Laserscan*>::iterator end =
        std::set_difference(before.begin(), before.end(), after.begin(), after.end(), needsDelete.begin());

    for (auto it = needsDelete.begin(); it != end; ++it) {
      bufferContent_.erase(*it);
    }
  }

  // reset free indexes.
  freeIndexes.clear();
  nextFree = 0;

  // get already used/occupied indexes.

  std::vector<int32_t> usedIndexes;
  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) usedIndexes.push_back(it->second.index);

  std::sort(usedIndexes.begin(), usedIndexes.end());
  usedIndexes.push_back(maxScans_);

  for (int32_t j = 0; j < usedIndexes[0]; ++j) freeIndexes.push_back(j);
  for (uint32_t i = 0; i < usedIndexes.size() - 1; ++i) {
    for (int32_t j = usedIndexes[i] + 1; j < usedIndexes[i + 1]; ++j) freeIndexes.push_back(j);
  }

  loadedScans = 0;

  fillBuffers(priorPoints, priorLabels, priorIndexes_);
  fillBuffers(pastPoints, pastLabels, pastIndexes_);

  glow::_CheckGlError(__FILE__, __LINE__);

  std::cout << "Loaded " << loadedScans << " of total " << priorPoints_.size() + pastPoints_.size() << " scans."
            << std::endl;
  //  std::cout << "memcpy: " << memcpy_time << " s / " << total_time << " s." << std::endl;

  updateGL();
}

void Viewport::fillBuffers(const std::vector<PointcloudPtr>& points, const std::vector<LabelsPtr>& labels,
                           std::vector<BufferInfo>& indexes) {
  for (uint32_t i = 0; i < points.size(); ++i) {
    if (bufferContent_.find(points[i].get()) == bufferContent_.end()) {
      if (nextFree == freeIndexes.size()) {
        std::cerr << "Warning: insufficient memory for scan." << std::endl;
        break;
      }

      int32_t index = freeIndexes[nextFree++];
      //      std::cout << index << std::endl;

      // not already loaded to buffer, need to transfer data to next free spot.
      uint32_t num_points = std::min(maxPointsPerScan_, points[i]->size());
      if (points[i]->size() >= maxPointsPerScan_) std::cerr << "warning: losing some points" << std::endl;

      std::vector<uint32_t> visible(num_points, 1);

      for (uint32_t j = 0; j < num_points; ++j) {
        if (std::find(mFilteredLabels.begin(), mFilteredLabels.end(), (*labels[i])[j]) != mFilteredLabels.end()) {
          visible[j] = 0;
        }
      }

      //      Stopwatch::tsic();

      BufferInfo info;
      info.index = index;
      info.size = num_points;
      info.scan = points[i].get();

      bufferContent_[points[i].get()] = info;
      bufPoses_[index] = points[i]->pose;
      bufPoints_.replace(index * maxPointsPerScan_, &points[i]->points[0], num_points);
      if (points[i]->hasRemissions())
        bufRemissions_.replace(index * maxPointsPerScan_, &points[i]->remissions[0], num_points);
      bufLabels_.replace(index * maxPointsPerScan_, &(*labels[i])[0], num_points);
      bufVisible_.replace(index * maxPointsPerScan_, &visible[0], num_points);

      //      memcpy_time += Stopwatch::toc();

      indexes.push_back(info);

      loadedScans += 1;
    } else {
      indexes.push_back(bufferContent_[points[i].get()]);
      //      indexes.back().scan = points[i].get();
    }
  }

  //  for (uint32_t i = 0; i < indexes.size(); ++i) {
  //    std::cout << indexes[i].index << ", " << indexes[i].scan << std::endl;
  //  }
}

void Viewport::setLabelColors(const std::map<uint32_t, glow::GlColor>& colors) {
  mLabelColors = colors;

  std::vector<uint8_t> label_colors(3 * 256);
  for (auto it = mLabelColors.begin(); it != mLabelColors.end(); ++it) {
    label_colors[3 * it->first] = it->second.R * 255;
    label_colors[3 * it->first + 1] = it->second.G * 255;
    label_colors[3 * it->first + 2] = it->second.B * 255;
  }
  texLabelColors_.assign(PixelFormat::RGB, PixelType::UNSIGNED_BYTE, &label_colors[0]);
}

void Viewport::setPointSize(int value) {
  pointSize_ = value;
  updateGL();
}

void Viewport::setMode(MODE mode) {
  mMode = mode;

  updateGL();
}

void Viewport::setFlags(int32_t flags) { mFlags = flags; }

void Viewport::setDrawingOption(const std::string& name, bool value) {
  drawingOption_[name] = value;
  updateGL();
}

void Viewport::setMinRange(float range) { minRange_ = range; }

void Viewport::setMaxRange(float range) { maxRange_ = range; }

void Viewport::setFilteredLabels(const std::vector<uint32_t>& labels) {
  std::vector<uint32_t> labels_before = mFilteredLabels;
  mFilteredLabels = labels;
  std::sort(mFilteredLabels.begin(), mFilteredLabels.end());

  //  std::vector<uint32_t> difference(std::max(labels_before.size(), labels.size()));
  //  auto end = std::set_difference(labels_before.begin(), labels_before.end(), mFilteredLabels.begin(),
  //                                 mFilteredLabels.end(), difference.begin());
  //
  //  for (auto it = difference.begin(); it != end; ++it) setLabelVisibility(*it, 1);  // now visible again.
  //
  //  end = std::set_difference(mFilteredLabels.begin(), mFilteredLabels.end(), labels_before.begin(),
  //  labels_before.end(),
  //                            difference.begin());
  //
  //  for (auto it = difference.begin(); it != end; ++it) setLabelVisibility(*it, 0);  // now invisible.

  updateGL();
}

void Viewport::setScanIndex(uint32_t idx) {
  singleScanIdx_ = idx;
  updateGL();
}

void Viewport::initializeGL() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LINE_SMOOTH);

  mCamera.lookAt(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
}

void Viewport::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  // set projection matrix
  float fov = radians(45.0f);
  float aspect = float(w) / float(h);

  projection_ = glPerspective(fov, aspect, 0.1f, 2000.0f);
}

void Viewport::paintGL() {
  glow::_CheckGlError(__FILE__, __LINE__);

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(pointSize_);

  model_ = Eigen::Matrix4f::Identity();
  view_ = mCamera.matrix();

  mvp_ = projection_ * view_ * conversion_;

  if (drawingOption_["coordinate axis"]) {
    // coordinateSytem_->pose = Eigen::Matrix4f::Identity();
    ScopedBinder<GlProgram> program_binder(prgDrawPose_);
    ScopedBinder<GlVertexArray> vao_binder(vao_no_points_);

    prgDrawPose_.setUniform(mvp_);
    prgDrawPose_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
    prgDrawPose_.setUniform(GlUniform<float>("size", 1.0f));

    glDrawArrays(GL_POINTS, 0, 1);
  }

  if (drawingOption_["show points"]) {
    glPointSize(pointSize_);

    ScopedBinder<GlProgram> program_binder(prgDrawPoints_);
    ScopedBinder<GlVertexArray> vao_binder(vao_points_);

    prgDrawPoints_.setUniform(GlUniform<bool>("useRemission", drawingOption_["remission"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("useColor", drawingOption_["color"]));
    prgDrawPoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
    prgDrawPoints_.setUniform(GlUniform<float>("minRange", minRange_));

    //    bool showSingleScan = drawingOption_["single scan"];

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    if (priorPoints_.size() > 0) {
      Eigen::Matrix4f anchor_pose = priorPoints_.back()->pose;
      //      Eigen::Matrix4f anchor_pose = Eigen::Matrix4f::Identity();
      if (drawingOption_["show train"]) {
        drawPoints(anchor_pose, priorIndexes_);
      } else {
        drawPoints(anchor_pose, priorIndexes_);
        drawPoints(anchor_pose, pastIndexes_);
      }
    }
    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();
  }

  if (drawingOption_["show voxels"]) {
    ScopedBinder<GlProgram> program_binder(prgDrawVoxels_);

    if (drawingOption_["show train"])
      vao_prior_voxels_.bind();
    else
      vao_past_voxels_.bind();

    //    bool showSingleScan = drawingOption_["single scan"];

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    mvp_ = projection_ * view_ * conversion_;
    prgDrawVoxels_.setUniform(mvp_);
    prgDrawVoxels_.setUniform(GlUniform<vec3>("lightPos", vec3(-10, 0, 10)));
    Eigen::Vector4f viewpos_rose = conversion_.inverse() * mCamera.getPosition();

    prgDrawVoxels_.setUniform(GlUniform<vec3>("viewPos", vec3(viewpos_rose.x(), viewpos_rose.y(), viewpos_rose.z())));

    if (drawingOption_["show train"])
      glDrawArrays(GL_POINTS, 0, bufPriorVoxels_.size());
    else
      glDrawArrays(GL_POINTS, 0, bufPastVoxels_.size());

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();

    if (drawingOption_["show train"])
      vao_prior_voxels_.release();
    else
      vao_past_voxels_.release();
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

void Viewport::drawPoints(const Eigen::Matrix4f& anchor_pose, const std::vector<BufferInfo>& indexes) {
  for (auto it = indexes.begin(); it != indexes.end(); ++it) {
    //      if (showSingleScan && (it->first != points_[singleScanIdx_].get())) continue;

    Eigen::Matrix4f pose = anchor_pose.inverse() * it->scan->pose;
    //    pose = it->scan->pose;

    prgDrawPoints_.setUniform(GlUniform<Eigen::Matrix4f>("pose", pose));
    mvp_ = projection_ * view_ * conversion_ * pose;

    prgDrawPoints_.setUniform(mvp_);

    glDrawArrays(GL_POINTS, it->index * maxPointsPerScan_, it->size);
  }
}

// std::ostream& operator<<(std::ostream& os, const vec2& v) {
//  os << "(" << v.x << ", " << v.y << ")";
//  return os;
//}

void Viewport::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.

  mChangeCamera = false;

  if (mCamera.mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                           resolveKeyboardModifier(event->modifiers()))) {
    timer_.start(1. / 30.);
    mChangeCamera = true;

    return;
  }
}

void Viewport::mouseReleaseEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    timer_.stop();
    updateGL();
    if (mCamera.mouseReleased(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                              resolveKeyboardModifier(event->modifiers()))) {
      timer_.stop();
      updateGL();  // get the last action.

      return;
    }
  }
}

void Viewport::mouseMoveEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (mChangeCamera) {
    if (mCamera.mouseMoved(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                           resolveKeyboardModifier(event->modifiers()))) {
      return;
    }
  }
}

void Viewport::keyPressEvent(QKeyEvent* event) { event->ignore(); }

glow::GlCamera::KeyboardModifier Viewport::resolveKeyboardModifier(Qt::KeyboardModifiers modifiers) {
  // currently only single button presses are supported.
  GlCamera::KeyboardModifier modifier = GlCamera::KeyboardModifier::None;

  if (modifiers & Qt::ControlModifier)
    modifier = GlCamera::KeyboardModifier::CtrlDown;
  else if (modifiers & Qt::ShiftModifier)
    modifier = GlCamera::KeyboardModifier::ShiftDown;
  else if (modifiers & Qt::AltModifier)
    modifier = GlCamera::KeyboardModifier::AltDown;

  return modifier;
}

glow::GlCamera::MouseButton Viewport::resolveMouseButton(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  GlCamera::MouseButton btn = GlCamera::MouseButton::NoButton;

  if (button & Qt::LeftButton)
    btn = GlCamera::MouseButton::LeftButton;
  else if (button & Qt::RightButton)
    btn = GlCamera::MouseButton::RightButton;
  else if (button & Qt::MiddleButton)
    btn = GlCamera::MouseButton::MiddleButton;

  return btn;
}
