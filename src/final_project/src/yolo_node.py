import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from yolov5 import YOLOv5

class YOLOImageDetection:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('yolo_image_detection_node', anonymous=True)
        
        # 初始化YOLOv5模型
        self.model = YOLOv5("yolov5s.pt")  # 使用yolov5s.pt模型，您可以换成其他版本
        
        # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像
        self.bridge = CvBridge()

        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # 创建发布带检测框图像的ROS话题
        self.image_pub = rospy.Publisher("/yolo/image_with_boxes", Image, queue_size=10)
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用YOLOv5进行目标检测
            results = self.model(cv_image)
            
            # 获取检测框的信息：框坐标、类别和置信度
            boxes = results.xywh[0].cpu().numpy()  # 获取边界框的xywh坐标
            classes = results.names  # 获取类别名称
            confidences = results.conf[0].cpu().numpy()  # 获取检测框的置信度
            
            # 在图像上绘制检测框
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = map(int, box[:4])  # 将坐标转换为整数
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绘制绿色矩形框
                label = f"{classes[int(box[4])]}: {confidences[i]:.2f}"  # 类别和置信度
                cv2.putText(cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 将处理后的图像转为ROS图像消息
            output_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 发布带检测框的图像
            self.image_pub.publish(output_image)
        
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        # 创建YOLO图像检测节点
        yolo_detector = YOLOImageDetection()
        
        # 保持节点持续运行
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass
