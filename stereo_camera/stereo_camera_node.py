import cv2
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StereoCameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')
        
        # 创建发布者
        self.left_publisher = self.create_publisher(Image, '/camera/left', 10)
        self.right_publisher = self.create_publisher(Image, '/camera/right', 10)
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error("无法打开摄像头！")
                raise RuntimeError("无法打开摄像头")
        
        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 创建定时器以固定频率发布图像
        timer_period = 0.033  # 约30Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("双目摄像头发布节点已启动")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture video")
            return
            
        # 分割左右图像
        left_frame = frame[:, :640, :]
        right_frame = frame[:, 640:, :]
        
        # 转换为ROS2消息并发布
        try:
            left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")
            
            left_msg.header.stamp = self.get_clock().now().to_msg()
            right_msg.header.stamp = left_msg.header.stamp
            
            self.left_publisher.publish(left_msg)
            self.right_publisher.publish(right_msg)
            
            # 可选：显示图像（调试用）
            cv2.imshow("Left Camera", left_frame)
            cv2.imshow("Right Camera", right_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"转换或发布图像时出错: {str(e)}")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StereoCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()