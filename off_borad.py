import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from std_msgs.msg import Float32, Bool, String

def is_middle_lane(hits):

    # True가 하나도 없으면 차선 자체가 없음
    if not any(hits):
        return False

    # 첫 True 위치
    first = next(i for i, v in enumerate(hits) if v)

    # 마지막 True 위치
    last = len(hits) - 1 - next(i for i, v in enumerate(reversed(hits)) if v)

    # 첫 True~마지막 True 사이에 False가 하나라도 있으면 → 점선(T,F,T)
    segment = hits[first:last+1]
    has_gap = False in segment

    return has_gap

def center_margin(cv_image, output, nonzerox, nonzeroy):

    center_margin =  int(cv_image.shape[1] / 2)
    center_left_margin = center_margin - 90
    center_right_margin = center_margin + 90

    # cv2.line(output, (center_left_margin, cv_image.shape[0]), (center_left_margin, 0), (255,0,0),3)
    # cv2.line(output, (center_right_margin, cv_image.shape[0]), (center_right_margin, 0), (255,0,0), 3)
    
    center_margin_mask = (nonzerox < center_left_margin) | (nonzerox > center_right_margin)
    nonzerox_filtered = nonzerox[center_margin_mask]
    nonzeroy_filtered = nonzeroy[center_margin_mask]

    return nonzerox_filtered, nonzeroy_filtered

def transform(img, h, w):

        src = np.float32([
            [100, 100],   # 좌상
            [539, 100],   # 우상
            [639, 479],   # 우하
            [  0, 479],   # 좌하
        ])

        dst = np.float32([
            [  0,   0],   # 좌상
            [639,   0],   # 우상
            [639, 479],   # 우하
            [  0, 479],   # 좌하
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        transform_view = cv2.warpPerspective(img, M, (w, h))

        return transform_view

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.lane_subscriber = self.create_subscription(Image, '/lane_image_raw',self.callback, 10)
        self.deg_pub = self.create_publisher(Float32, '/deg', 10)
        self.angle_pub = self.create_publisher(Float32, '/angle', 10)
        self.deg_valid_pub = self.create_publisher(Bool, '/deg_valid', 10)
        self.lane_pub = self.create_publisher(String, '/lane_text', 10)

        self.left_is_middle  = False
        self.right_is_middle = False

        self.left_middle_until  = 0.0
        self.right_middle_until = 0.0

        self.prev_lane_text = 'none'

        self.cv_bridge = CvBridge()
        self.get_logger().info('Camera subscriber started.')
    
    def callback(self, lane_msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(lane_msg,"bgr8")
        blur = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 90])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel3 = np.ones((3,3), np.uint8)
        kernel5 = np.ones((5,5), np.uint8)

        # 노이즈 제거, 안정화
        mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN,  kernel3, iterations=1)
        mask = cv2.morphologyEx(mask,       cv2.MORPH_CLOSE, kernel5, iterations=1)

        # 원근 변환
        h, w = cv_image.shape[:2]
        top_view = transform(mask, h, w)

        # 3채널 확장
        output = np.dstack((top_view, top_view, top_view)) * 255

        # 0 > 픽셀좌표 추출
        nonzero = (top_view > 0).nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        histogram = np.sum(top_view[int(top_view.shape[0] / 2):, :], axis=0)

        # # 히스토그램 시각화 확인
        # hist_img = np.zeros((300, len(histogram), 3), dtype=np.uint8)

        # max_val = np.max(histogram)
        # if max_val == 0: 
        #     max_val = 1
        # normalized = (histogram / max_val) * 300

        # for x, val in enumerate(normalized):
        #     cv2.line(hist_img, (x, 300), (x, 300 - int(val)), (0, 255, 0), 1)

        # cv2.imshow("Histogram", hist_img)

        mid_point = int(histogram.shape[0] / 2)
        
        # 히스토그램 가로 축 범위
        start_leftX = np.argmax(histogram[:mid_point])
        start_rightX = np.argmax(histogram[mid_point:]) + mid_point

        # 변수 선언
        windows = 10

        windows_margin = 60

        nonzerox_filtered, nonzeroy_filtered = center_margin(cv_image, output, nonzerox, nonzeroy)

        hegiht = int(top_view.shape[0] / windows)

        min_num_pixel = 200

        win_left_lane = []
        win_right_lane = []
        
        win_left_dot = []
        win_right_dot = []

        win_left_hit = []
        win_right_hit = []

        # 중앙 슬라이딩 윈도우
        center_margin_length = 90
        
        c_y = int(cv_image.shape[0])
        c_x = int(cv_image.shape[1] / 2)

        roi = top_view[0:c_y, c_x - center_margin_length:c_x + center_margin_length]
        roi = roi.nonzero()
        roi_y = np.array(roi[0])
        roi_x = np.array(roi[1])

        roi_x = roi_x + (c_x - center_margin_length)

        # 슬라이딩 윈도우 차선 감지
        for i in range(0, 7):
            win_y_low = top_view.shape[0] - (i + 1) * hegiht
            win_y_high = top_view.shape[0] - (i + 0) * hegiht
            win_leftx_min = start_leftX - windows_margin
            win_leftx_max = start_leftX + windows_margin
            win_rightx_min = start_rightX - windows_margin
            win_rightx_max = start_rightX + windows_margin

            cv2.rectangle(output, (win_leftx_min, win_y_low), (win_leftx_max, win_y_high), (0, 165, 255), 2)
            cv2.rectangle(output, (win_rightx_min, win_y_low), (win_rightx_max, win_y_high), (0, 165, 255), 2)

            # 윈도우 영역 True 값 추출
            win_left_true = ((nonzeroy_filtered >= win_y_low) & (nonzeroy_filtered <= win_y_high) & (nonzerox_filtered >= win_leftx_min) & (
                nonzerox_filtered <= win_leftx_max)).nonzero()[0]
            win_right_true = ((nonzeroy_filtered >= win_y_low) & (nonzeroy_filtered <= win_y_high) & (nonzerox_filtered >= win_rightx_min) & (
                nonzerox_filtered <= win_rightx_max)).nonzero()[0]

            # 추출된 True 값 리스트 저장
            win_left_lane.append(win_left_true)
            win_right_lane.append(win_right_true)
            
            hit_left  = len(win_left_true) >= min_num_pixel
            hit_right = len(win_right_true) >= min_num_pixel

            win_left_hit.append(hit_left)
            win_right_hit.append(hit_right)

            # 최소값 보다 크다면 평균을 구하고 현재 픽셀값으로 변경
            if len(win_left_true) > min_num_pixel:
                start_leftX = int(np.mean(nonzerox_filtered[win_left_true]))
            if len(win_right_true) > min_num_pixel:
                start_rightX = int(np.mean(nonzerox_filtered[win_right_true]))

            # 왼쪽 윈도우 평균값 도트 표시
            if len(win_left_true) > 0:
                mean_leftx  = int(np.mean(nonzerox_filtered[win_left_true]))
                mean_lefty  = int(np.mean(nonzeroy_filtered[win_left_true]))
                win_left_dot.append((int(mean_leftx), int(mean_lefty)))
                # cv2.circle(output, (mean_leftx, mean_lefty), 5, (154, 250, 0), -1) # 오른쪽 점
                
            # 윈도우 평균값 도트 표시
            if len(win_right_true) > 0:
                mean_rightx = int(np.mean(nonzerox_filtered[win_right_true]))
                mean_righty = int(np.mean(nonzeroy_filtered[win_right_true]))
                win_right_dot.append((int(mean_rightx), int(mean_righty)))
                # cv2.circle(output, (mean_rightx, mean_righty), 5, (154, 250, 0), -1) # 오른쪽 점

        # 차선 시각화
        win_left_lane = np.concatenate(win_left_lane)
        win_right_lane = np.concatenate(win_right_lane)

        leftx, lefty = nonzerox_filtered[win_left_lane], nonzeroy_filtered[win_left_lane]
        rightx, righty = nonzerox_filtered[win_right_lane], nonzeroy_filtered[win_right_lane]

        output[lefty, leftx] = [255, 0, 0]
        output[righty, rightx] = [0, 0, 255]

        left_gap  = is_middle_lane(win_left_hit)
        right_gap = is_middle_lane(win_right_hit)

        now = time.time()

        if left_gap:
            self.left_middle_until = now + 3.0
        if right_gap:
            self.right_middle_until = now + 3.0

        # 현재 시간 기준으로 상태 업데이트
        self.left_is_middle  = (now < self.left_middle_until)
        self.right_is_middle = (now < self.right_middle_until)

        lane_text = 'none'

        if self.left_is_middle and not self.right_is_middle:
            # print('2st lane')   # 왼쪽만 중앙선
            lane_text = '2st lane'

        elif self.right_is_middle and not self.left_is_middle:
            # print('1st lane')    # 오른쪽만 중앙선
            lane_text = '1st lane'

        elif self.left_is_middle and self.right_is_middle:
            # print('thinking...')
            lane_text = 'thinking'   # 둘 다 중앙선으로 보이는 애매 상태

        else:
            # print('lane not found')
            lane_text = 'none'       # 차선 못 찾음

        # 🔹 thinking일 때는 이전 프레임 라벨을 그대로 사용
        if lane_text == 'thinking':
            lane_to_publish = self.prev_lane_text
        else:
            lane_to_publish = lane_text
            # thinking이 아닐 때만 prev 갱신
            self.prev_lane_text = lane_text

        msg = String()
        msg.data = lane_to_publish
        self.lane_pub.publish(msg)
            
        # 중앙선으로 판단된 라인 색칠
        if self.left_is_middle:
            output[lefty, leftx] = [0, 255, 255]
        if self.right_is_middle:
            output[righty, rightx] = [0, 255, 255]

        steering_x = None

        cut_num = 10
        c_t_height = int(c_y / cut_num)
        
        win_center_lane = []
        
        # 양쪽 차선 조향각
        if len(win_left_dot) > 0 and len(win_right_dot) > 0:
            left_x_lane = [left_x[0] for left_x in win_left_dot]
            right_x_lane = [right_x[0] for right_x in win_right_dot]
            
            left_x_lane = np.mean(left_x_lane)
            right_x_lane = np.mean(right_x_lane)
            
            robot_front_y = int(top_view.shape[0] * 0.8)
            robot_front_x = int(top_view.shape[1] / 2)
            steering_y = int(top_view.shape[0] * 0.8)
            steering_x = int((left_x_lane + right_x_lane) / 2)
            
            cv2.circle(output, (robot_front_x, robot_front_y), 7, (0, 255, 255), -1)
            cv2.circle(output, (steering_x, steering_y), 7, (0, 165, 255), 2)

            angle = steering_x - robot_front_x
            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.angle_pub.publish(angle_msg)
            # print("current angle is : %d" % int(angle_msg.data))

            valid_msg = Bool()
            valid_msg.data = False
            self.deg_valid_pub.publish(valid_msg)
            
        # 한쪽 차선 조향각
        elif len(win_left_dot) == 0 and len(win_right_dot) > 0 or len(win_left_dot) > 0 and len(win_right_dot) == 0:
            robot_front_y = int(top_view.shape[0] * 0.8)
            robot_front_x = int(top_view.shape[1] / 2)

            for j in range(2, 6):
                c_win_y_low = c_y - (j + 1) * c_t_height
                c_win_y_high = c_y - (j + 0) * c_t_height
                c_win_min = c_x - center_margin_length
                c_win_max = c_x + center_margin_length

                cv2.rectangle(output, (c_win_min, c_win_y_low), (c_win_max, c_win_y_high), (255, 191, 0), 2)

                win_center_true = ((c_win_min <= roi_x) & (c_win_max >= roi_x) & (c_win_y_low <= roi_y) & (c_win_y_high >= roi_y)).nonzero()[0]
                
                if len(win_center_true) > min_num_pixel:
                    mean_centerx = float(np.mean(roi_x[win_center_true]))
                    mean_centery = float(np.mean(roi_y[win_center_true]))
                    win_center_lane.append((mean_centerx, mean_centery))
                    cx = roi_x[win_center_true]
                    cy = roi_y[win_center_true]
                    output[cy, cx] = [255, 191, 0]   # center line color (yellow)
            
            # 기울기
            if len(win_center_lane) >= 2:
                pts = np.array(sorted(win_center_lane, key=lambda p: -p[1]), dtype=np.float32)
                (x1, y1), (x2, y2) = pts[:2]
                angle_deg = int(np.degrees(np.arctan2(y1 - y2, x2 - x1)))
                deg = 90 - angle_deg
                deg_msg = Float32()
                deg_msg.data = float(deg)
                self.deg_pub.publish(deg_msg)
                # print("deg is : %d" % int(deg_msg.data))
                valid_msg = Bool()
                valid_msg.data = True
                self.deg_valid_pub.publish(valid_msg)

            else:
                valid_msg = Bool()
                valid_msg.data = False
                self.deg_valid_pub.publish(valid_msg)
                        
        else:
            pass
        
        cv2.putText(cv_image, lane_text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3, cv2.LINE_AA)
        cv2.imshow('output', output)
        cv2.imshow('cv_image', cv_image)
        cv2.waitKey(1)
       

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()