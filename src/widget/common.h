#ifndef SRC_WIDGET_COMMON_H_
#define SRC_WIDGET_COMMON_H_

#include <data/Pointcloud.h>
#include <memory>
#include "data/geometry.h"
#include <glow/glutil.h>

typedef std::shared_ptr<Laserscan> PointcloudPtr;
typedef std::shared_ptr<std::vector<uint32_t>> LabelsPtr;
typedef std::shared_ptr<std::vector<float>> ColorsPtr;


struct LabeledVoxel{
  public:
    glow::vec3 position;  // lower left corner
    uint32_t label;       // majority voted label.
};

#endif /* SRC_WIDGET_COMMON_H_ */
