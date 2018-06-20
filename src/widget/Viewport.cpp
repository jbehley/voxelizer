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
}

void Viewport::initVertexBuffers() {
  vao_points_.setVertexAttribute(0, bufPoints_, 3, AttributeType::FLOAT, false, sizeof(Point3f), nullptr);
  vao_points_.setVertexAttribute(1, bufRemissions_, 1, AttributeType::FLOAT, false, sizeof(float), nullptr);
  vao_points_.setVertexAttribute(2, bufLabels_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
  vao_points_.setVertexAttribute(3, bufVisible_, 1, AttributeType::UNSIGNED_INT, false, sizeof(uint32_t), nullptr);
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

void Viewport::setPoints(const std::vector<PointcloudPtr>& p, std::vector<LabelsPtr>& l) {
  std::cout << "Setting points..." << std::flush;

  glow::_CheckGlError(__FILE__, __LINE__);

  // FIXME: improve usage of resources:
  //   Use transform feedback to get points inside the tile.

  // determine which labels need to be updated.
  std::vector<uint32_t> indexes;
  index_difference(labels_, l, indexes);

  for (auto index : indexes) {
    if (bufferContent_.find(points_[index].get()) == bufferContent_.end()) continue;
    const BufferInfo& info = bufferContent_[points_[index].get()];

    // replace label information with labels from GPU.
    bufLabels_.get(*labels_[index], info.index * maxPointsPerScan_, info.size);
  }

  points_ = p;
  labels_ = l;
  glow::_CheckGlError(__FILE__, __LINE__);

  // TODO: on unload => fetch labels and update labels in file.

  {
    // first remove entries using a set_difference
    std::vector<Laserscan*> before;
    std::vector<Laserscan*> after;
    for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) before.push_back(it->first);
    for (auto it = p.begin(); it != p.end(); ++it) after.push_back(it->get());

    std::sort(before.begin(), before.end());
    std::sort(after.begin(), after.end());

    std::vector<Laserscan*> needsDelete(before.size());
    std::vector<Laserscan*>::iterator end =
        std::set_difference(before.begin(), before.end(), after.begin(), after.end(), needsDelete.begin());

    for (auto it = needsDelete.begin(); it != end; ++it) {
      bufferContent_.erase(*it);
    }
  }

  std::vector<int32_t> usedIndexes;
  for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
    usedIndexes.push_back(it->second.index);
  }

  std::sort(usedIndexes.begin(), usedIndexes.end());
  usedIndexes.push_back(maxScans_);

  std::vector<int32_t> freeIndexes;
  for (int32_t j = 0; j < usedIndexes[0]; ++j) freeIndexes.push_back(j);
  for (uint32_t i = 0; i < usedIndexes.size() - 1; ++i) {
    for (int32_t j = usedIndexes[i] + 1; j < usedIndexes[i + 1]; ++j) freeIndexes.push_back(j);
  }

  uint32_t nextFree = 0;
  uint32_t loadedScans = 0;
  float memcpy_time = 0.0f;

  Stopwatch::tic();
  for (uint32_t i = 0; i < points_.size(); ++i) {
    if (bufferContent_.find(points_[i].get()) == bufferContent_.end()) {
      if (nextFree == freeIndexes.size()) {
        std::cerr << "Warning: insufficient memory for scan." << std::endl;
        break;
      }

      int32_t index = freeIndexes[nextFree++];
      //      std::cout << index << std::endl;

      // not already loaded to buffer, need to transfer data to next free spot.
      uint32_t num_points = std::min(maxPointsPerScan_, points_[i]->size());
      if (points_[i]->size() >= maxPointsPerScan_) std::cerr << "warning: losing some points" << std::endl;

      std::vector<uint32_t> visible(num_points, 1);

      for (uint32_t j = 0; j < num_points; ++j) {
        if (std::find(mFilteredLabels.begin(), mFilteredLabels.end(), (*labels_[i])[j]) != mFilteredLabels.end()) {
          visible[j] = 0;
        }
      }

      Stopwatch::tic();

      BufferInfo info;
      info.index = index;
      info.size = num_points;

      bufferContent_[points_[i].get()] = info;
      bufPoses_[index] = points_[i]->pose;
      bufPoints_.replace(index * maxPointsPerScan_, &points_[i]->points[0], num_points);
      if (points_[i]->hasRemissions())
        bufRemissions_.replace(index * maxPointsPerScan_, &points_[i]->remissions[0], num_points);
      bufLabels_.replace(index * maxPointsPerScan_, &(*labels_[i])[0], num_points);
      bufVisible_.replace(index * maxPointsPerScan_, &visible[0], num_points);

      memcpy_time += Stopwatch::toc();

      loadedScans += 1;
    }
  }

  float total_time = Stopwatch::toc();

  glow::_CheckGlError(__FILE__, __LINE__);

  std::cout << "Loaded " << loadedScans << " of total " << points_.size() << " scans." << std::endl;
  std::cout << "memcpy: " << memcpy_time << " s / " << total_time << " s." << std::endl;

  updateGL();
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

  if (points_.size() > 0) {
    glPointSize(pointSize_);

    ScopedBinder<GlProgram> program_binder(prgDrawPoints_);
    ScopedBinder<GlVertexArray> vao_binder(vao_points_);

    prgDrawPoints_.setUniform(GlUniform<bool>("useRemission", drawingOption_["remission"]));
    prgDrawPoints_.setUniform(GlUniform<bool>("useColor", drawingOption_["color"]));
    prgDrawPoints_.setUniform(GlUniform<float>("maxRange", maxRange_));
    prgDrawPoints_.setUniform(GlUniform<float>("minRange", minRange_));

    prgDrawPoints_.setUniform(GlUniform<bool>("showAllPoints", drawingOption_["show all points"]));
    prgDrawPoints_.setUniform(GlUniform<int32_t>("heightMap", 1));

    bool showSingleScan = drawingOption_["single scan"];

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.bind();

    for (auto it = bufferContent_.begin(); it != bufferContent_.end(); ++it) {
      if (showSingleScan && (it->first != points_[singleScanIdx_].get())) continue;
      prgDrawPoints_.setUniform(GlUniform<Eigen::Matrix4f>("pose", it->first->pose));

      mvp_ = projection_ * view_ * conversion_ * it->first->pose;
      prgDrawPoints_.setUniform(mvp_);

      glDrawArrays(GL_POINTS, it->second.index * maxPointsPerScan_, it->second.size);
    }

    glActiveTexture(GL_TEXTURE0);
    texLabelColors_.release();
  }

  glow::_CheckGlError(__FILE__, __LINE__);
}

std::ostream& operator<<(std::ostream& os, const vec2& v) {
  os << "(" << v.x << ", " << v.y << ")";
  return os;
}

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
