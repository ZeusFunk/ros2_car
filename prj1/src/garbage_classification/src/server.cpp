#include "rclcpp/rclcpp.hpp"
#include "garbage_classification/srv/garbage_classification.hpp"

using namespace std::placeholders;

class GarbageClassifier : public rclcpp::Node {
public:
  GarbageClassifier() : Node("garbage_classification_server") {
    service_ = this->create_service<garbage_classification::srv::GarbageClassification>(
      "garbage_classification",
      std::bind(&GarbageClassifier::handle_request, this, _1, _2));
    
    // 初始化垃圾分类数据库
    classification_db_ = {
      {"apple_core", {"厨余垃圾", "放入绿色垃圾桶，建议沥干水分后投放"}},
      {"battery", {"有害垃圾", "放入红色垃圾桶，避免破损和高温"}},
      {"plastic_bottle", {"可回收垃圾", "清洗压扁后放入蓝色垃圾桶"}},
      {"newspaper", {"可回收垃圾", "捆扎整齐后放入蓝色垃圾桶"}},
      {"ceramic_bowl", {"其他垃圾", "放入灰色垃圾桶"}}
    };
  }

private:
  void handle_request(
    const garbage_classification::srv::GarbageClassification::Request::SharedPtr req,
    garbage_classification::srv::GarbageClassification::Response::SharedPtr res) {
    
    auto it = classification_db_.find(req->name);
    if (it != classification_db_.end()) {
      res->category = it->second.first;
      res->guide = it->second.second;
    } else {
      res->category = "未知分类";
      res->guide = "请咨询当地垃圾分类管理部门";
    }
  }

  rclcpp::Service<garbage_classification::srv::GarbageClassification>::SharedPtr service_;
  std::unordered_map<std::string, std::pair<std::string, std::string>> classification_db_;
};

int main(int argc, char ​**argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GarbageClassifier>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}