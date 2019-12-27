# track human using yolo and siamRPN
import cv2
import torch
from time import time
from DaSiamRPN.run_SiamRPN import SiamRPN_init, SiamRPN_track
from DaSiamRPN.net import SiamRPNvot
from DaSiamRPN.utils import cxy_wh_2_rect, rect_2_cxy_wh
from Yolov3.detector import detector


def iou(bbox1, bbox2):
    boxA = [bbox1[0], bbox1[1], bbox1[0]+bbox1[2], bbox1[1]+bbox1[3]]
    boxB = [bbox2[0], bbox2[1], bbox2[0]+bbox2[2], bbox2[1]+bbox2[3]]

    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

    iou = interArea / float(boxAArea + boxBArea - interArea)

    return iou


class HumanTracker(object):
    """
        Use DaSiamRPN and Yolov3 to track human
    """
    def __init__(self, count=5):
        # init net
        print('init tracker ...')
        start = time()
        self.net = SiamRPNvot()
        self.net.load_state_dict(torch.load('/home/car/projects/catkin_ws/src/tracking/scripts/DaSiamRPN/SiamRPNVOT.model'))
        self.net.eval().cuda()

        self.yolo = detector()
        self.human_detected = False
        self.tracking = False
        self.iou_threshold = 0.5
        self.max_count = count
        self.count = count
        print('tracker initialize done, cost time {}'.format(time()-start))

    def track(self, img):
        self.count -= 1

        # get siamrpn tracker's result
        state = SiamRPN_track(self.state, img)
        res = cxy_wh_2_rect(state['target_pos'], state['target_sz'])
        res = [int(l) for l in res]

        if self.count <= 0:
            # use detector to examine
            human = self.yolo.detect_human(img)
            if len(human) == 0:
                self.human_detected = False
                # if no detection, return sot result
                return res
            self.count = self.max_count

            # compare iou between detect result and tracking result
            bbox = [human[0], human[1], human[2]-human[0], human[3]-human[1]]
            bbox = [int(l) for l in bbox]
            if iou(bbox, res) < self.iou_threshold:
                self.init_SOT(bbox, img)
                return bbox
            else:
                return res
        return res

    def init_SOT(self, bbox, img):
        start = time()
        self.tracking = True
        self.human_detected = True
        target_pos, target_sz = rect_2_cxy_wh(bbox)
        self.state = SiamRPN_init(img, target_pos, target_sz, self.net)
        print('SOT init done, cost time: {}s'.format(time()-start))

    def detect(self, img):
        # detect human and return position
        human = self.yolo.detect_human(img)
        if len(human) == 0:
            return (False, None)
        else:
            init_box = [human[0], human[1], human[2]-human[0], human[3]-human[1]]
            return (True, init_box)


def main():
    # a camera demo
    h = HumanTracker()
    cap = cv2.VideoCapture(0)
    _, img = cap.read()
    while True:
        detected, res = h.detect(img)
        if not detected:
            _, img = cap.read()
            cv2.imshow('detecting', img)
            cv2.waitKey(1)
            continue
        else:
            cv2.rectangle(img, (res[0], res[1]), (res[2], res[3]), (255, 0, 0))
            cv2.imshow('detecting', img)
            h.init_SOT(res, img)
            break

    while True:
        _, img = cap.read()
        bbox = h.track(img)
        cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (255, 0, 0))
        cv2.imshow('tracking', img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
