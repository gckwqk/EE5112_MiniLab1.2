#!/usr/bin/env python3
"""Camera-only HSV colour candidates for EE5112 Task 3.

No ground-truth coordinates or motion commands are used. A detection here is
not a mission FOUND event. Created with AI assistance; add group authorship.
"""
import json
import math

import cv2
import numpy as np


COLOURS = ('Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Purple', 'Black')
DEFAULTS = {
    'roi_top_fraction': 0.45,
    'roi_bottom_fraction': 1.0,
    'min_area_px': 35.0,
    'max_area_fraction': 0.12,
    'min_aspect_ratio': 0.4,
    'max_aspect_ratio': 2.2,
    'min_fill_ratio': 0.5,
    'min_solidity': 0.75,
    'morphology_kernel': 3,
    'reject_border_blobs': True,
    'stable_frames': 3,
    'track_min_iou': 0.10,
    'max_frame_gap': 0.4,
    'black_min_area_px': 60.0,
    'black_max_area_fraction': 0.04,
    'black_min_aspect_ratio': 0.60,
    'black_max_aspect_ratio': 1.80,
    'black_min_fill_ratio': 0.65,
    'black_min_solidity': 0.85,
    'black_min_contrast': 20.0,
    # Each group of six: Hlow, Slow, Vlow, Hhigh, Shigh, Vhigh.
    # Hue is 0..179 for OpenCV's 8-bit HSV; S/V are 0..255.
    'hsv.red': [0, 90, 45, 9, 255, 255, 170, 90, 45, 179, 255, 255],
    'hsv.orange': [10, 90, 45, 22, 255, 255],
    'hsv.yellow': [23, 90, 45, 38, 255, 255],
    'hsv.green': [39, 90, 45, 88, 255, 255],
    'hsv.blue': [89, 90, 45, 130, 255, 255],
    'hsv.purple': [131, 90, 45, 169, 255, 255],
    'hsv.black': [0, 0, 0, 179, 110, 65],
}


def intersection_over_union(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    intersection = max(0, min(ax+aw, bx+bw)-max(ax,bx)) * max(0, min(ay+ah,by+bh)-max(ay,by))
    return intersection / (aw*ah + bw*bh - intersection)


class ColourDetector:
    def __init__(self, parameters=None):
        self.p = dict(DEFAULTS)
        self.p.update(parameters or {})
        p = self.p
        if not 0 <= p['roi_top_fraction'] < p['roi_bottom_fraction'] <= 1:
            raise ValueError('ROI fractions must satisfy 0 <= top < bottom <= 1.')
        if p['stable_frames'] < 1 or p['max_frame_gap'] <= 0:
            raise ValueError('stable_frames and max_frame_gap must be positive.')
        size = p['morphology_kernel']
        if not isinstance(size, int) or size < 1 or size % 2 != 1:
            raise ValueError('morphology_kernel must be a positive odd integer.')
        if not 0 <= p['track_min_iou'] <= 1:
            raise ValueError('track_min_iou must be in [0, 1].')
        for prefix in ('', 'black_'):
            if not 0 < p[prefix+'max_area_fraction'] <= 1 or p[prefix+'min_area_px'] <= 0:
                raise ValueError('Invalid area limits.')
            if not 0 < p[prefix+'min_aspect_ratio'] <= p[prefix+'max_aspect_ratio']:
                raise ValueError('Invalid aspect limits.')
            for name in ('min_fill_ratio', 'min_solidity'):
                if not 0 <= p[prefix+name] <= 1:
                    raise ValueError('Fill and solidity thresholds must be in [0, 1].')
        for colour in COLOURS:
            values = p['hsv.'+colour.lower()]
            if not values or len(values) % 6:
                raise ValueError(f'{colour}: expected groups of six HSV integers.')
            for i in range(0,len(values),6):
                low,high=values[i:i+3],values[i+3:i+6]
                if not all(isinstance(x,int) for x in low+high):
                    raise ValueError('HSV bounds must be integers.')
                if not all(0 <= a <= b <= cap for a,b,cap in zip(low,high,(179,255,255))):
                    raise ValueError(f'{colour}: invalid HSV bounds.')
        self.kernel = np.ones((size,size),np.uint8)
        self.previous = {}
        self.last_stamp = None

    def reset(self):
        self.previous = {}
        self.last_stamp = None

    def detect(self, bgr, stamp):
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError('Expected a uint8 BGR image with three channels.')
        if not math.isfinite(stamp):
            raise ValueError('Timestamp must be finite.')
        height,width=bgr.shape[:2]
        top=int(height*self.p['roi_top_fraction'])
        bottom=int(height*self.p['roi_bottom_fraction'])
        if bottom-top < 3 or width < 3:
            raise ValueError('Image ROI too small.')
        if self.last_stamp is not None and (stamp <= self.last_stamp or stamp-self.last_stamp > self.p['max_frame_gap']):
            self.previous = {}
        self.last_stamp=stamp
        hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
        roi=hsv[top:bottom]
        masks={}
        candidates=[]
        current={}
        for colour in COLOURS:
            mask=np.zeros(roi.shape[:2],np.uint8)
            values=self.p['hsv.'+colour.lower()]
            for i in range(0,len(values),6):
                mask=cv2.bitwise_or(mask,cv2.inRange(
                    roi,np.array(values[i:i+3],np.uint8),np.array(values[i+3:i+6],np.uint8)))
            mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernel)
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.kernel)
            full_mask=np.zeros((height,width),np.uint8)
            full_mask[top:bottom]=mask
            masks[colour]=full_mask
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            prefix='black_' if colour=='Black' else ''
            accepted=[]
            for contour in contours:
                area=float(cv2.contourArea(contour))
                if not self.p[prefix+'min_area_px'] <= area <= self.p[prefix+'max_area_fraction']*width*height:
                    continue
                x,y,w,h=cv2.boundingRect(contour)
                if self.p['reject_border_blobs'] and (x<=0 or y<=0 or x+w>=width or y+h>=bottom-top):
                    continue
                aspect=w/h
                fill=area/(w*h)
                hull_area=float(cv2.contourArea(cv2.convexHull(contour)))
                solidity=area/hull_area if hull_area>0 else 0
                if not self.p[prefix+'min_aspect_ratio'] <= aspect <= self.p[prefix+'max_aspect_ratio']:
                    continue
                if fill<self.p[prefix+'min_fill_ratio'] or solidity<self.p[prefix+'min_solidity']:
                    continue
                if colour=='Black':
                    # Require a dark component surrounded by a brighter ring.
                    # This reduces some false positives; it cannot prove cube identity.
                    margin=max(3,round(min(w,h)*.15))
                    x0,y0=max(0,x-margin),max(0,y-margin)
                    x1,y1=min(width,x+w+margin),min(bottom-top,y+h+margin)
                    patch=roi[y0:y1,x0:x1,2]
                    ring=np.ones(patch.shape,bool)
                    ring[y-y0:y+h-y0,x-x0:x+w-x0]=False
                    inside=roi[y:y+h,x:x+w,2][mask[y:y+h,x:x+w]>0]
                    if not ring.any() or inside.size==0:
                        continue
                    contrast=float(np.median(patch[ring]))-float(np.median(inside))
                    if contrast<self.p['black_min_contrast']:
                        continue
                accepted.append({'colour':colour,'bbox_xywh':[int(x),int(y+top),int(w),int(h)],
                                 'centre_uv':[float(x+w/2),float(y+top+h/2)],
                                 'area_px':area,'fill_ratio':round(fill,3),'solidity':round(solidity,3)})
            if accepted:
                # One block of each colour exists in this arena. Keep the largest
                # qualifying candidate, but require spatial continuity to stabilise.
                best=max(accepted,key=lambda d:d['area_px'])
                previous=self.previous.get(colour)
                count=1
                if previous and intersection_over_union(previous['bbox_xywh'],best['bbox_xywh'])>=self.p['track_min_iou']:
                    count=min(previous['support_frames']+1,self.p['stable_frames'])
                best['support_frames']=count
                best['stable']=count>=self.p['stable_frames']
                candidates.append(best)
                current[colour]=best
        self.previous=current  # Disappeared colours are removed immediately.
        return candidates,masks,(top,bottom)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge, CvBridgeError

    class ColourDetectorNode(Node):
        def __init__(self):
            super().__init__('colour_detector')
            descriptor=ParameterDescriptor(read_only=True)
            for name,value in DEFAULTS.items():
                self.declare_parameter(name,value,descriptor)
            for name,value in {
                'image_topic':'/camera/color/image_raw',
                'detections_topic':'/detected_colours',
                'debug_image_topic':'/colour_detector/debug_image',
                'mask_topic':'/colour_detector/mask',
                'mask_colour':'Black',
            }.items():
                self.declare_parameter(name,value,descriptor)
            self.detector=ColourDetector({k:self.get_parameter(k).value for k in DEFAULTS})
            self.mask_colour=self.get_parameter('mask_colour').value.title()
            if self.mask_colour not in COLOURS:
                raise ValueError('mask_colour must be one of the seven block colours.')
            self.bridge=CvBridge()
            self.results_pub=self.create_publisher(String,self.get_parameter('detections_topic').value,1)
            self.debug_pub=self.create_publisher(Image,self.get_parameter('debug_image_topic').value,1)
            self.mask_pub=self.create_publisher(Image,self.get_parameter('mask_topic').value,1)
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
            self.sub=self.create_subscription(Image,self.get_parameter('image_topic').value,self.on_image,qos)
            self.last_colours=None
            self.get_logger().info('Camera colour detector ready on '+self.get_parameter('image_topic').value)

        def publish_results(self,msg,candidates,status):
            payload={'schema_version':1,'status':status,
                     'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},
                     'frame_id':msg.header.frame_id,'image_width':msg.width,'image_height':msg.height,
                     'detections':[d for d in candidates if d['stable']], 'candidates':candidates}
            output=String()
            output.data=json.dumps(payload,separators=(',',':'),allow_nan=False)
            self.results_pub.publish(output)

        def on_image(self,msg):
            try:
                image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                candidates,masks,(top,bottom)=self.detector.detect(image,stamp)
            except (CvBridgeError,cv2.error,ValueError) as error:
                self.detector.reset()
                self.publish_results(msg,[],'image_error')
                self.get_logger().error(str(error),throttle_duration_sec=2.0)
                return
            self.publish_results(msg,candidates,'ok')
            debug=image.copy()
            cv2.line(debug,(0,top),(debug.shape[1]-1,top),(180,180,180),1)
            if bottom<debug.shape[0]:
                cv2.line(debug,(0,bottom-1),(debug.shape[1]-1,bottom-1),(180,180,180),1)
            for d in candidates:
                x,y,w,h=d['bbox_xywh']
                pen=(0,255,0) if d['stable'] else (0,180,255)
                cv2.rectangle(debug,(x,y),(x+w-1,y+h-1),pen,2)
                label=d['colour']+(' detected' if d['stable'] else ' candidate')
                cv2.putText(debug,label,(x,max(15,y-6)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),3)
                cv2.putText(debug,label,(x,max(15,y-6)),cv2.FONT_HERSHEY_SIMPLEX,.45,pen,1)
            debug_msg=self.bridge.cv2_to_imgmsg(debug,encoding='bgr8')
            debug_msg.header=msg.header
            self.debug_pub.publish(debug_msg)
            mask_msg=self.bridge.cv2_to_imgmsg(masks[self.mask_colour],encoding='mono8')
            mask_msg.header=msg.header
            self.mask_pub.publish(mask_msg)
            colours=tuple(d['colour'] for d in candidates if d['stable'])
            if colours!=self.last_colours:
                self.get_logger().info('[DETECT] colours='+(', '.join(colours) if colours else 'none'))
                self.last_colours=colours

    rclpy.init(args=args)
    node=None
    try:
        node=ColourDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
