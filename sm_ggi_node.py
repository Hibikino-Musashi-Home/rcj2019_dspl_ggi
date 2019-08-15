#!/usr/bin/env python
# -*- coding: utf-8 -*-




#==================================================

## @file
## @author Kentaro NAKAMURA
## @brief RCJ2019 Go Get It のステートマシンROSノード

#==================================================




import sys
import roslib
import time

sys.path.append(roslib.packages.get_pkg_dir("hma_common_pkg") + "/script/import/common")
from common_import import *
from common_global import *
from common_param import *


sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libmysql import *
from libsound import *
from libopencv import *
from libtf import *
from libsub import *
from libaction import *

sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib/eol")
from libspeech import *


sys.path.append(roslib.packages.get_pkg_dir("hma_nlp_pkg") + "/script/src/lib")


if GP_ROBOT != "none":
    sys.path.append(roslib.packages.get_pkg_dir("hma_" + GP_ROBOT + "_pkg") + "/script/import/robot")
    from robot_import import *
    from robot_global import *
    from robot_param import *


    sys.path.append(roslib.packages.get_pkg_dir("hma_" + GP_ROBOT + "_pkg") + "/script/src/lib")
    from librobotpub import *
    from librobotsub import *
    from librobotaction import *
    from librobotspeech import *
    from librobotnav import *
    from libmap import *

    sys.path.append(roslib.packages.get_pkg_dir("hma_" + GP_ROBOT + "_pkg") + "/script/src/common")
    from common_function import *


# 品詞分解に使うライブラリ（/home/hsr??/nltk_dataが生成されるので注意！）
import nltk
nltk.download('punkt')
nltk.download('averaged_perceptron_tagger')


#==================================================

# グローバル

#==================================================

#GP_OPERATION_POINT = Pose2D(0.0, 0.0, 0.0) # オペレーションポイントDEBUG
GP_OPERATION_POINT = Pose2D(1.3, 0.91, 0.04) # オペレーションポイント（競技前日に公開）

GP_COUNTER_PAN = [20.0, -10.0 , 20.0]


#==================================================

## @class 初期化ステート

#==================================================
class Init(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])


        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):
        rospy.set_param(rospy.get_name() + "/count_rec_obj", 0)         # 認識した物体のカウント
        rospy.set_param(rospy.get_name() + "/count_get_obj", 0)         # 取得した物体のカウント
        rospy.set_param(rospy.get_name() + "/rec_object_point", [(0.0,0.0,0.0),(0.0,0.0,0.0),(0.0,0.0,0.0)])   # 物体の座標
        rospy.set_param(rospy.get_name() + "/rec_object_attr", [["bottle","shelf"],[],[]])    # 物体の属性
        rospy.set_param(rospy.get_name() + "/get_object_idx", [0,1,2])     # 取りに行く物体のインデックス
        rospy.set_param(rospy.get_name() + "/rec_miss", 0)              # 各物体の把持に失敗した数
        rospy.set_param(rospy.get_name() + "/arm_lift_joint", 0)        # 
        rospy.set_param(rospy.get_name() + "/head_pan_joint", 0)        # 
        

        return "normal"




#==================================================

## @class スタート待ちステート

#==================================================
class WaitStart(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech


        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        self._libsound.soundEffectSync(0)
        self._libsound.soundEffectSync(0)
        txt = ["準備します．", "I'm preparing to start."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)


        self._librobotaction.wholeBodyMoveToGo()
        self._librobotaction.gripperCommand(1.2)


        txt = ["", "Push my hand to start."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

        while not rospy.is_shutdown():
            if self._librobotsub.armSwitch():
                break

        txt = ["スタートします．", "Let's go!"]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)


        return "normal"




#==================================================

## @class 標準スタートステート

#==================================================
class NormalStart(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])


        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "normal"



######################
# Training Phase
######################

#==================================================

## @class ドアを開く

#==================================================
class OpenDoor(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub
    ):
        smach.State.__init__(self, outcomes = ["normal", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub

        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        #==================================================

        # ドアオープン

        #==================================================
        #"""
        call(["rosrun", "hma_common_pkg", "detect_door_node.py"])


        txt = ["", "Let's rock in roll!"]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

        rospy.sleep(1.0)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)
        self._librobotpub.omniBaseVel(0.1, 0.0, 0.0)
        rospy.sleep(0.5)

        print "aa"

        # オペレーションポイントにナビゲーション
        self._librobotnav.goPlace(
            GP_OPERATION_POINT
        )
        #"""


        return "normal"





#==================================================

## @class オペレーションポイントでの行動

#==================================================
class GoToStartPoint(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libmap
    ):
        smach.State.__init__(self, outcomes = ["train", "test", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libmap = libmap

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._robot_descriptor = self._libutil.getRobotDescriptor()


        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        count_rec_obj = rospy.get_param(rospy.get_name() + "/count_rec_obj", 0)     # 認識した物体のカウント
        count_get_obj = rospy.get_param(rospy.get_name() + "/count_get_obj", 0)     # 取得した物体のカウント



        #########################################################################
        # DEBUG オペレーションポイントの座標記録
        """
        map2base_footprint = self._libtf.getPoseWithoutOffset(
            self._robot_descriptor["FRAME_MAP"],
            self._robot_descriptor["FRAME_BASE_FOOTPRINT"]
        )

        x = map2base_footprint.position.x
        y = map2base_footprint.position.y
        z = map2base_footprint.position.z

        (roll, pitch, yaw) = euler_from_quaternion(
            [
                map2base_footprint.orientation.x,
                map2base_footprint.orientation.y,
                map2base_footprint.orientation.z,
                map2base_footprint.orientation.w
            ]
        )

        GP_OPERATION_POINT = (x,y,yaw)   # オペレーションポイント
        """
        #########################################################################



        self._librobotaction.wholeBodyMoveToGo()
        self._librobotaction.gripperCommand(1.2)

        txt = ["スタートします．", "Please stand in front of me."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        rospy.sleep(1.0)

        # 回転しながらYOLOで人物認識
        cmd = "rostopic pub -1 /darknet_ros_wrapper_node/start std_msgs/Empty"
        Popen(cmd.split(" "))
        person_flag = False

        while not rospy.is_shutdown():

            #TODO 認識精度を上げる
            obj_rec_res_person = self._libsub.getObjRecRes()
            for i in range(obj_rec_res_person.amount):
                if obj_rec_res_person.is_valid_pose[i] == True:
                    if obj_rec_res_person.id[i] in "person":
                        #距離計算
                        distance = math.sqrt(obj_rec_res_person.pose[i].position.x**2 + obj_rec_res_person.pose[i].position.y**2)
                        if distance < 1.5:
                            person_flag = True

                            # map座標に変換する
                            pose_from_map = self._libtf.getPoseWithOffset(
                                self._robot_descriptor["FRAME_MAP"],
                                self._robot_descriptor["FRAME_BASE_FOOTPRINT"],
                                "guest_person",
                                obj_rec_res_person.pose[i]
                            )
                            rospy.set_param(rospy.get_name() + "/guest_info/from_map/x",  pose_from_map.position.x )
                            rospy.set_param(rospy.get_name() + "/guest_info/from_map/y",  pose_from_map.position.y )

            if person_flag is True:
                break            

            rospy.sleep(0.5)

        cmd = "rostopic pub -1 /darknet_ros_wrapper_node/stop std_msgs/Empty"
        Popen(cmd.split(" "))
        rospy.sleep(0.2)


        txt = ["スタートします．", "I found person."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        rospy.sleep(1.0)


        if count_rec_obj < 3:

            txt = ["スタートします．", "training phase start."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
            rospy.sleep(1.0)

            return "train"

        elif count_get_obj < 3:

            txt = ["スタートします．", "test phase start."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
            rospy.sleep(1.0)

            return "test"




#==================================================

## @class 指示者についていく

#==================================================
class FollowMe(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libmap
    ):
        smach.State.__init__(self, outcomes = ["normal", "end", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libmap = libmap

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._robot_descriptor = self._libutil.getRobotDescriptor()

        self._pub_2d_pose_estimate = rospy.Publisher(
            "/laser_2d_correct_pose",
            PoseWithCovarianceStamped,
            queue_size = 1
        )

        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================
    
    ## @fn オンラインマップからオフラインマップに切り替える関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def switchOnline2Offline(
        self
    ):
        cmd = "rosnode kill /hector_slam"
        Popen(cmd.split(" "))
        rospy.sleep(1.0)

        cmd = "rosrun tmc_pose_integrator pose_integrator "
        cmd += "laser_2d_pose:=/laser_2d_pose "
        cmd += "__name:=pose_integrator"
        Popen(cmd.split(" "))
        rospy.sleep(1.0)

        # 自己位置合わせ
        pubt_2d_pose_estimate = PoseWithCovarianceStamped()
        pubt_2d_pose_estimate.header.stamp = rospy.Time.now()
        pubt_2d_pose_estimate.header.frame_id = "map"

        pubt_2d_pose_estimate.pose.pose = Pose(
            Point(
                GP_OPERATION_POINT.x,
                GP_OPERATION_POINT.y,
                0.0
            ),
            Quaternion(0.0, 0.0, 0.0, 1.0)
        )

        pubt_2d_pose_estimate.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.06853891945200942
        ]

        self._pub_2d_pose_estimate.publish(pubt_2d_pose_estimate)
        

        return



    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        count_rec_obj = rospy.get_param(rospy.get_name() + "/count_rec_obj", 0)     # 認識した物体のカウント


        if count_rec_obj == 0:
            txt = ["物体の色や特徴", "Let's go to learn the first object."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        elif count_rec_obj == 1:
            txt = ["物体の色や特徴", "Let's go to learn the second object."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        elif count_rec_obj == 2:
            txt = ["物体の色や特徴", "Let's go to learn the third object."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        else:
            txt = ["物体の色や特徴", "I will go to the operation point."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

            # オペレーションポイントにナビゲーション
            self._librobotnav.goPlace(
                GP_OPERATION_POINT
            )

            self.switchOnline2Offline()
            rospy.sleep(3.0)

            return "end"



        #########################################################################
        # TODO Follow me 中に指示者が数秒間停止したらロボットも停止して次のステートに進む

        cmd = "rostopic pub -1 /follow_operator_node/start std_msgs/Empty"
        Popen(cmd.split(" "))
        cmd = "rostopic pub -1 /follow_target_node/start std_msgs/Empty"
        Popen(cmd.split(" "))
        rospy.sleep(1.0)

        while not rospy.is_shutdown():

            # HSRのヘッドディスプレイにbbox付き画像を表示
            #cv_mat_rgb_bbox = self._libsub.subRGBDRGB()
            self._librobotpub.HSRDisplay(1)#, img=cv_mat_rgb_bbox)

            if self._librobotsub.armSwitch():
                cmd = "rostopic pub -1 /follow_operator_node/end std_msgs/Empty"
                Popen(cmd.split(" "))
                cmd = "rostopic pub -1 /follow_target_node/end std_msgs/Empty"
                Popen(cmd.split(" "))
                rospy.sleep(1.0)

                break

        #########################################################################



        return "normal"






#==================================================

## @class 画像認識と音声認識と位置取得でオブジェクトをラベル付け

#==================================================
class RecObject(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libspeech
    ):
        smach.State.__init__(self, outcomes = ["normal", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libspeech = libspeech

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._robot_descriptor = self._libutil.getRobotDescriptor()


        return




    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        ################################# test ###################################
        obj_name_test = [["apple","fruit"],["computer","PC"],["doll","mouse"]]
        obj_place_test = [["kitchen"],["table"],["desk"]]
        obj_color_test = [["red","orange"],["blue","purple"],["yellow","orange"]]
        ##########################################################################

        count_rec_obj = rospy.get_param(rospy.get_name() + "/count_rec_obj", 0)     # 認識した物体のカウント
        rec_object_attr = rospy.get_param(rospy.get_name() + "/rec_object_attr", [[],[],[]])    # 物体の属性
        rec_object_point = rospy.get_param(rospy.get_name() + "/rec_object_point", [[],[],[]])  # 物体の座標

        tmp_rec_object_attr = []  # 認識した物体の属性



        #########################################################################
        # TODO 停止地点の座標をサブスクライブして記録

        map2base_footprint = self._libtf.getPoseWithoutOffset(
            self._robot_descriptor["FRAME_MAP"],
            self._robot_descriptor["FRAME_BASE_FOOTPRINT"]
        )

        x = map2base_footprint.position.x
        y = map2base_footprint.position.y
        z = map2base_footprint.position.z

        (roll, pitch, yaw) = euler_from_quaternion(
            [
                map2base_footprint.orientation.x,
                map2base_footprint.orientation.y,
                map2base_footprint.orientation.z,
                map2base_footprint.orientation.w
            ]
        )

        rec_object_point[count_rec_obj] = (x,y,yaw)


        #########################################################################



        txt = ["物体の色や特徴", "Prease show me the object."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)



        #########################################################################
        # TODO YOLO9000によるBBの範囲内で物体の色を認識

        # YOLOv2(COCO)で物体の名前を認識結果から補足する
        rec_object = []

        # YOLOの起動
        cmd = "rostopic pub -1 /darknet_ros_wrapper_node/start std_msgs/Empty"
        Popen(cmd.split(" "))
        rospy.sleep(1.0)

        txt = ["物体の色や特徴", "Put the object in the blue square."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        rospy.sleep(1.0)

        rec_time = 10.0
        rec_names = []
        time_now = time.time()
        while not rospy.is_shutdown():

            if time.time() - time_now > rec_time:

                break

            # HSRのヘッドディスプレイにbbox付き画像を表示
            cv_mat_rgb_bbox = self._libsub.getDarknetBBoxSMIRGBDRGBD()
            self._librobotpub.HSRDisplay(6, img=cv_mat_rgb_bbox)

            tmp_rec_object = []
            # 認識したオブジェクトの情報と中心点からの距離を計算
            obj_rec_res = self._libsub.getObjRecRes()

            for i in range(obj_rec_res.amount):
                if obj_rec_res.id[i] != "person":
                    id = obj_rec_res.id[i]
                    x = obj_rec_res.x[i]
                    y = obj_rec_res.y[i]
                    w = obj_rec_res.w[i]
                    h = obj_rec_res.h[i]
                    tmp_rec_object.append([id, abs(240-(x+w/2))+abs(320-(y+h/2))])

            if len(tmp_rec_object) == 0:

                continue

            # 中心点から最も近いオブジェクトを学習オブジェクトとする
            tmp_rec_object = sorted(tmp_rec_object, key=lambda x:x[-1])
            rec_object.append(tmp_rec_object[0][0])

            rospy.sleep(0.5)


        # YOLOの終了
        cmd = "rostopic pub -1 /darknet_ros_wrapper_node/stop std_msgs/Empty"
        Popen(cmd.split(" "))
        rospy.sleep(0.2)

        # 学習オブジェクトの属性
        for i in reversed(range(len(rec_object))):
            if rec_object[i] not in tmp_rec_object_attr:
                tmp_rec_object_attr.append(rec_object[i])

        # HSRのヘッドディスプレイに記憶した物体の画像を表示
        tmp_rec_object_img = self._libsub.subRGBDRGB()
        self._librobotpub.HSRDisplay(3, img=tmp_rec_object_img)

        #########################################################################



        txt = ["物体の名前", "Prease tell me its feature."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        rospy.sleep(0.5)



        #########################################################################
        # TODO 物体の名前を音声認識して記録

        while not rospy.is_shutdown():
            try:
                txt_and_score = self._librobotspeech.recSpeech("", "", 0.0, 10.0, 5.0)

                print("========================================")
                print("text = " + str(txt_and_score[0]["TXT"]))
                print("score = " + str(txt_and_score[0]["SCORE"]))

                tmp_rec_object_attr.append(txt_and_score[0]["TXT"])
                tmp_rec_object_attr.append(txt_and_score[1]["TXT"])
                tmp_rec_object_attr.append(txt_and_score[2]["TXT"])

                if txt_and_score[0]["TXT"] == "that's all":
                    self._librobotpub.HSRDisplay(0, "Complete.")
                    rospy.sleep(1.0)
                    break
                else:
                    txt = txt_and_score[0]["TXT"] + ", " + txt_and_score[1]["TXT"] + ", " + txt_and_score[2]["TXT"]
                    self._librobotpub.HSRDisplay(0, txt)
                    rospy.sleep(1.0)

            except HMAExTimeout:
                txt = ["場所の記録", "Please speak again."]
                self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

        #########################################################################



        txt = ["物体の色や特徴", "I recorded the object's feature."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

        rec_object_attr[count_rec_obj] = tmp_rec_object_attr
        count_rec_obj += 1

        print("count_rec_obj = " + str(count_rec_obj))
        print("rec_object_point = " + str(rec_object_point))
        print("rec_object_attr = " + str(rec_object_attr))

        rospy.set_param(rospy.get_name() + "/count_rec_obj", count_rec_obj)         # 認識した物体のカウント
        rospy.set_param(rospy.get_name() + "/rec_object_attr", rec_object_attr)     # 物体の属性
        rospy.set_param(rospy.get_name() + "/rec_object_point", rec_object_point)     # 物体の属性

        # 記憶オブジェクトの画像を保存
        cv2.imwrite(roslib.packages.get_pkg_dir("hma_hsr_ggi_pkg") + "/io/rec_object_img/" + str(count_rec_obj) + ".png", tmp_rec_object_img)


        return "normal"




######################
# Test Phase
######################

#==================================================

## @class オーダーを聞く

#==================================================
class TakeOrder(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libspeech
    ):
        smach.State.__init__(self, outcomes = ["normal", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libspeech = libspeech

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._robot_descriptor = self._libutil.getRobotDescriptor()

        return




    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================

    ## @fn 品詞分解する関数 by kanamaru
    ## @brief
    ## @param
    ## @return

    #==================================================
    def morphologicalAnalysis(
        self,
        cmd
    ):
        #==================================================
        # コマンド入力
        #==================================================
        cmd = cmd.replace("bring me ", "")
        words = nltk.word_tokenize(cmd)

        analysis = nltk.pos_tag(words)

        obj = []
        NN = [s for s in analysis if 'NN' in s]
        for i in range(len(NN)):
            obj.append(NN[i][0])

        JJ = [s for s in analysis if 'JJ' in s]
        for i in range(len(JJ)):
            obj.append(JJ[i][0])
        print("test obj = " + str(obj))

        return obj



    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        ######################  test  ######################
        # 「Bring me a juice near the standing person. 」
        # 「Bring me a red object in front of the TV. 」
        # 「Bring me a green tea on a table. 」
        ####################################################

        count_get_obj = rospy.get_param(rospy.get_name() + "/count_get_obj", 0)     # 認識した物体のカウント
        rec_object_attr = rospy.get_param(rospy.get_name() + "/rec_object_attr", [[],[],[]])    # 物体の属性
        get_object_idx = rospy.get_param(rospy.get_name() + "/get_object_idx", [0,1,2])  # 取りに行く物体のインデックス

        txt = ["物体の色や特徴", "Please tell me what objects to get."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        rospy.sleep(1.0)

        tmp_get_object_attr = []

        print("get_object_idx = " + str(get_object_idx))
        
        #########################################################################
        # TODO 音声認識

        while not rospy.is_shutdown():

            try:
                txt_and_score = self._librobotspeech.recSpeech("", "", 0.0, 10.0, 20.0)

                print("========================================")
                print("text = " + str(txt_and_score[0]["TXT"]))
                print("score = " + str(txt_and_score[0]["SCORE"]))

                tmp_get_object_attr.append(txt_and_score[0]["TXT"])
                tmp_get_object_attr.append(txt_and_score[1]["TXT"])

                break

            except HMAExTimeout:
                txt = ["場所の記録", "sorry. I couldn't hear your voice."]
                self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
                rospy.sleep(0.5)
                txt = ["場所の記録", "Please say again."]
                self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
                rospy.sleep(1.0)

        #########################################################################



        #########################################################################
        # TODO 品詞分解したTest Commandと認識リストと比較

        # 品詞分解
        get_object_attr = []
        for i in range(len(tmp_get_object_attr)):
            tmp = self.morphologicalAnalysis(tmp_get_object_attr[i])
            for j in range(len(tmp)):
                if tmp[j] not in get_object_attr:
                    get_object_attr.append(tmp[j])

        print("get_object_attr = " + str(get_object_attr))
        #########################################################################



        #########################################################################
        # TODO 合致度の高いオブジェクトから順に画像を使って意思確認

        # 合致度計算
        point = [0,0,0]
        for word in get_object_attr:
            for i in xrange(len(rec_object_attr)):
                for j in xrange(len(rec_object_attr[i])):
                    if word in rec_object_attr[i][j]:
                        point[i] += 1

        print("point = " + str(point))

        dic = {0:point[0], 1:point[1], 2:point[2]}
        dic_sorted = sorted(dic.items(), key=lambda x:-x[1])

        dic_chosen = []
        for i in range(len(dic_sorted)):
            if dic_sorted[i][1] > 0:
                dic_chosen.append(dic_sorted[i])

        # 画像をioファイルから取得
        print("#### rec_object_img ####")
        rec_object_img = []
        directory = roslib.packages.get_pkg_dir("hma_hsr_ggi_pkg") + "/io/rec_object_img"
        for root, dirs, files in os.walk(directory):
            for file in files:
                if ".png" in file:
                    rec_object_img.append(cv2.imread(os.path.join(root, file)))
                    print(file)
        print("len rec_object_img = " + str(len(rec_object_img)))
        print("########################")

        # 合致度降順でオペレータにオブジェクト画像を提示して取得物体を確認
        exception = 0
        for i in range(len(dic_chosen)):
            index = dic_chosen[i][0]
            # HSRのヘッドディスプレイに記憶した物体の画像を表示
            self._librobotpub.HSRDisplay(4, img=rec_object_img[index])
            txt = ["物体の色や特徴", "Should I bring this object, is it correct?"]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
            """
            rospy.sleep(1.0)
            txt = ["物体の色や特徴", "Please answer yes it is or no it isn't."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
            rospy.sleep(1.0)
            """

            print("index = " + str(index))

            while not rospy.is_shutdown():

                try:
                    txt_and_score = self._librobotspeech.recSpeech("", "", 0.0, 10.0, 10.0)

                    print("========================================")
                    print("text = " + str(txt_and_score[0]["TXT"]))
                    print("score = " + str(txt_and_score[0]["SCORE"]))

                    tmp_get_object_attr.append(txt_and_score[0]["TXT"])
                    tmp_get_object_attr.append(txt_and_score[1]["TXT"])

                    break

                except HMAExTimeout:
                    txt = ["場所の記録", "Please say again."]
                    self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

            if "yes" in txt_and_score[0]["TXT"] or "yes" in txt_and_score[1]["TXT"] or "yes" in txt_and_score[2]["TXT"] or "yes" in txt_and_score[3]["TXT"] or "yes" in txt_and_score[4]["TXT"]:
                txt = ["物体の色や特徴", "OK, I'll bring it."]
                self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

                get_object_idx[count_get_obj] = int(index)
                exception += 1 

                break

        if exception == 0:
            txt = ["物体の色や特徴", "Sorry, I could not understand your command."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
            rospy.sleep(1.0)

            return "loop"

        #########################################################################



        rospy.set_param(rospy.get_name() + "/get_object_idx", get_object_idx)     # 取りに行く物体のインデックス

        print("get_object_idx = " + str(get_object_idx))


        return "normal"




#==================================================

## @class オーダーされたものを取りに行く

#==================================================
class GoToObject(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libmap,
        common_function
    ):
        smach.State.__init__(self, outcomes = ["normal", "end", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libmap = libmap

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._common_function = common_function
        self._robot_descriptor = self._libutil.getRobotDescriptor()

        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        count_get_obj = rospy.get_param(rospy.get_name() + "/count_get_obj", 0)             # 取得した物体のカウント
        rec_object_point = rospy.get_param(rospy.get_name() + "/rec_object_point", [[],[],[]])    # 物体の座標
        rec_object_attr = rospy.get_param(rospy.get_name() + "/rec_object_attr", [[],[],[]])    # 物体の属性
        get_object_idx = rospy.get_param(rospy.get_name() + "/get_object_idx", [0,1,2])  # 取りに行く物体のインデックス
        rec_miss = rospy.get_param(rospy.get_name() + "/rec_miss")
        Arm_Lift_Joint = rospy.get_param(rospy.get_name() + "/arm_lift_joint")
        Head_Pan_Joint = rospy.get_param(rospy.get_name() + "/head_pan_joint")

        # 指定されたオブジェクトの情報
        get_object_attr = rec_object_attr[get_object_idx[count_get_obj]]
        get_object_point = rec_object_point[get_object_idx[count_get_obj]]

        # shel
        for i in xrange(len(get_object_attr)):
            print get_object_attr[i]
            if get_object_attr[i] in "shelf":
                rospy.set_param(rospy.get_name() + "/shelf/flag", True)
                break
            else:
                rospy.set_param(rospy.get_name() + "/shelf/flag", False)



        #########################################################################
        # TODO 移動

        point_x = get_object_point[0]
        point_y = get_object_point[1]
        point_yaw = get_object_point[2]

        shelf_flag = rospy.get_param(rospy.get_name() + "/shelf/flag", False)
        if shelf_flag is False:
            self._librobotnav.goPlace(
                Pose2D(point_x, point_y, point_yaw),
                GP_APPROACH_ANGLE,
                [
                    "arm_lift_joint",
                    "arm_flex_joint",
                    "arm_roll_joint",
                    "wrist_flex_joint",
                    "wrist_roll_joint",
                    "head_pan_joint",
                    "head_tilt_joint"
                ],
                [
                    0.5,
                    -22.0,
                    50.0, 
                    -75.0,
                    0.0,
                    -30.0,
                    -50.0
                ]
            )
        else:
            self._librobotnav.goPlace(
                Pose2D(point_x, point_y, point_yaw),
                GP_APPROACH_ANGLE,
                [
                    "arm_lift_joint",
                    "arm_flex_joint",
                    "arm_roll_joint",
                    "wrist_flex_joint",
                    "wrist_roll_joint",
                    "head_pan_joint",
                    "head_tilt_joint"
                ],
                [
                    0.68,
                    -22.0,
                    50.0, 
                    -75.0,
                    0.0,
                    -30.0,
                    -30.0
                ]
            )

        #########################################################################



        #########################################################################
        # TODO 物体把持

        # 回転
        """
        r = rospy.Rate(5)
        start_time = time.time()
        while not rospy.is_shutdown():
            self._librobotpub.omniBaseVel(
                0.0,
                0.0,
                (GP_COUNTER_PAN[rec_miss]/1) * G_DEG2RAD
            )
            if time.time() - start_time > 1:
                break
            r.sleep()
        """
        # 把持ポーズ＆物体認識

        print "arm_lift_joint" + str(Arm_Lift_Joint)
        print "head_pan_joint" + str(Head_Pan_Joint + GP_COUNTER_PAN[int(rec_miss)])

        # 把持物体の選択
        index = -1
        for j in xrange(30):
            if shelf_flag is False:
                obj_rec_res = self._common_function.recObj(
                    [
                        "arm_lift_joint",
                        "arm_flex_joint",
                        "arm_roll_joint",
                        "wrist_flex_joint",
                        "wrist_roll_joint",
                        "head_pan_joint",
                        "head_tilt_joint"
                    ],
                    [
                        0.5,
                        -22.0,
                        50.0, 
                        -75.0,
                        0.0,
                        -30.0,
                        -50.0
                    ]
                )
            else:
                obj_rec_res = self._common_function.recObj(
                    [
                        "arm_lift_joint",
                        "arm_flex_joint",
                        "arm_roll_joint",
                        "wrist_flex_joint",
                        "wrist_roll_joint",
                        "head_pan_joint",
                        "head_tilt_joint"
                    ],
                    [
                        0.68,
                        -22.0,
                        50.0, 
                        -75.0,
                        0.0,
                        -30.0,
                        -30.0
                    ]
                )

            print("====== get object ======")
            print("TAKE" + str(rec_miss))
            #print("get_object_attr = " + str(get_object_attr))
            #print("obj_rec_res.id = " + str(obj_rec_res.id))

            min_distance = 1000.0
            index = -1
            for i in xrange(obj_rec_res.amount):
                # valid_poseでバリデーション
                if obj_rec_res.is_valid_pose[i] is False:
                    continue

                # label（ID）でバリデーション
                for k in xrange(len(get_object_attr)):
                    if obj_rec_res.id[i] == str(get_object_attr[k]):
                        break
                else:
                    continue

                # 距離でバリデーション
                distance_x, distance_y = obj_rec_res.pose[i].position.x, obj_rec_res.pose[i].position.y
                distance = math.sqrt(pow(distance_x, 2) + pow(distance_y, 2))
                
                # 最低距離
                if distance > min_distance:
                    continue

                # 物体情報の格納
                min_distance = distance
                index = i

            if index != -1:
                break

        else:
            txt = ["", "I could not found the object."]
            self._librobotspeech.talkRequestNonSync(txt[GP_LANG], GP_LANG)

            # オペレーションポイントにナビゲーション
            self._librobotnav.goPlace(
                GP_OPERATION_POINT
            )

            return "end"
        

        #情報をセット
        rospy.set_param(rospy.get_name() + "/grasp_obj/id", str(obj_rec_res.id[index]))
        rospy.set_param(rospy.get_name() + "/grasp_obj/pose/x", obj_rec_res.pose[index].position.x)
        rospy.set_param(rospy.get_name() + "/grasp_obj/pose/y", obj_rec_res.pose[index].position.y)
        rospy.set_param(rospy.get_name() + "/grasp_obj/pose/z", obj_rec_res.pose[index].position.z)

        #########################################################################

        print("I could find the object !!!!")

        rospy.set_param(rospy.get_name() + "/rec_miss", 0)

        txt = ["物体の色や特徴", "I found a object."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        #rospy.sleep(1.0)

        return "normal"






#==================================================

## @class オーダーされたものを把持する

#==================================================
class TakeObject(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libmap,
        common_function
    ):
        smach.State.__init__(self, outcomes = ["normal", "move", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libmap = libmap

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._common_function = common_function
        self._robot_descriptor = self._libutil.getRobotDescriptor()

        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================
    
    ## @fn 把持点推定関数
    ## @brief
    ## @param loop_n ループ回数
    ## @param obj_id 物体id
    ## @param obj_pose 物体座標
    ## @param offset 物体検出領域のオフセット
    ## @return　obj_info　把持点推定結果　Flase 把持点推定失敗

    #==================================================
    def getGraspPoint(
        self,
        loop_n,
        obj_id,
        obj_pose,
        offset_x,
        offset_y,
        offset_z,
    ):
        for i in xrange(loop_n):
            obj_info = self._libaction.getObjGraspPoint(
                "base_link",
                obj_pose.position.x - offset_x,
                obj_pose.position.x + offset_x,
                obj_pose.position.y - offset_y,
                obj_pose.position.y + offset_y,
                obj_pose.position.z - offset_z,
                obj_pose.position.z + offset_z,
                obj_id,
                obj_pose
            )
            if obj_info.is_valid_grasp_point == True:
                break
        else:
            txt = ["", "I could not found the object."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

            return False
            

        # TFブロードキャスト
        self._libtf.setTransform(
            "grasp_obj",
            "base_link",
            obj_info.pose
        )


        return obj_info



    #==================================================
    
    ## @fn 物体把持関数（把持点推定なし）
    ## @brief
    ## @param loop_n ループ回数
    ## @param obj_info 物体情報
    ## @param place_h 物体が置かれている場所の高さ
    ## @param grasp_point 把持点の固定（Noneなら推定結果を使用）
    ## @param arm_up_val アームを上げる量 [deg]
    ## @param back_val バックする量 [m]
    ## @return　

    #==================================================
    def graspObj(
        self,
        obj_info,
        arm_up_val = 5.0,
        back_val = 0.15
    ):
        self._librobotaction.gripperCommand(1.2)

        # オフセットの算出
        offset_x = 0.0
        offset_y = 0.0
        offset_z = 0.0
        offset_theta = 0.0


        #==================================================

        # 正面から把持

        #==================================================
        offset_x, offset_y = self._libutil.rotateCoordinate(-0.2, 0.0, -GP_APPROACH_ANGLE)
        offset_z = 0.0

        self._librobotaction.wholeBodyMoveEndEffectorPose(
            95.0,
            20.0,
            "hand_palm_link",
            True,
            float(obj_info["pose_x"]) + offset_x,
            float(obj_info["pose_y"]) + offset_y,
            float(obj_info["pose_z"]) + offset_z,
            60.0 * G_DEG2RAD,
            -90.0 * G_DEG2RAD,
            90.0 * G_DEG2RAD,
            "base_link"
        )
        rospy.sleep(0.5)

        self._librobotaction.wholeBodyMoveEndEffectorPose(
            80.0,
            20.0,
            "hand_palm_link",
            True,
            0.0,
            0.0,
            -offset_x,
            0.0,
            0.0,
            0.0,
            "hand_palm_link"
        )

        # 把持
        rospy.sleep(0.1)
        self._librobotaction.gripperApplyForce(0.5)


        # 少しアームを上げる
        current_joint_positions = self._librobotsub.getJointPositions()
        self._librobotpub.wholeBodyMoveToArmPositions(
            current_joint_positions["arm_lift_joint"] + 0.05,
            current_joint_positions["arm_flex_joint"] + arm_up_val * G_DEG2RAD,
            current_joint_positions["arm_roll_joint"],
            current_joint_positions["wrist_flex_joint"],
            current_joint_positions["wrist_roll_joint"],
            1.0
        )
        rospy.sleep(0.5)

        # 少しバックする
        for i in xrange(int(back_val * 1000.0)):
            self._librobotpub.omniBaseVel(
                -0.1,
                0.0,
                0.0
            )
            rospy.sleep(0.01)


        return


    #==================================================
    
    ## @fn 物体把持関数（把持点推定あり）
    ## @brief
    ## @param loop_n ループ回数
    ## @param obj_info 物体情報
    ## @param place_h 物体が置かれている場所の高さ
    ## @param grasp_point 把持点の固定（Noneなら推定結果を使用）
    ## @param arm_up_val アームを上げる量 [deg]
    ## @param back_val バックする量 [m]
    ## @return　

    #==================================================
    def graspObjWithGetGraspPoint(
        self,
        obj_info,
        place_h,
        grasp_point = None,
        arm_up_val = 15.0,
        back_val = 0.15,
        lift_up_val = 0.1
    ):
        # オフセットの算出
        offset_x = 0.0
        offset_y = 0.0
        offset_z = 0.0
        offset_theta = 0.0

        if obj_info.grasp_point == GP_GRASP_TOP:
            # 場所の高さに応じて変更
            if place_h < 0.6:
                offset_x, offset_y = self._libutil.rotateCoordinate(0.02, 0.0, -GP_APPROACH_ANGLE)
                offset_z = obj_info.h + 0.15
            else:
                offset_x, offset_y = self._libutil.rotateCoordinate(0.0, 0.0, -GP_APPROACH_ANGLE)
                offset_z = obj_info.h + 0.05

            # offset theta
            _, _, yaw = self._libtf.quaternionToEuler(obj_info.pose.orientation)
            offset_theta = yaw * G_RAD2DEG

            if offset_theta < -110.0:
                offset_theta += 180.0
            elif offset_theta > 90.0:
                offset_theta -= 180.0


        elif obj_info.grasp_point == GP_GRASP_FRONT:
            offset_x, offset_y = self._libutil.rotateCoordinate(-0.15, 0.0, -GP_APPROACH_ANGLE)
            offset_z = -0.02


        #==================================================

        # 上から把持 

        #==================================================
        if obj_info.grasp_point == GP_GRASP_TOP:
            self._librobotaction.wholeBodyMoveEndEffectorPose(
                95.0,
                20.0,
                "hand_palm_link",
                True,
                float(obj_info.pose.position.x) + offset_x,
                float(obj_info.pose.position.y) + offset_y,
                float(obj_info.pose.position.z) + offset_z,
                0.0,
                180.0 * G_DEG2RAD,
                offset_theta * G_DEG2RAD,
                "base_link"
            )

            # 場所の高さに応じて変更
            current_joint_positions = self._librobotsub.getJointPositions()
            if place_h < 0.6:
                self._librobotpub.wholeBodyMoveToArmPositions(
                    current_joint_positions["arm_lift_joint"],
                    -106.0 * G_DEG2RAD,
                    current_joint_positions["arm_roll_joint"],
                    -74.0 * G_DEG2RAD,
                    current_joint_positions["wrist_roll_joint"],
                    0.5
                )
                rospy.sleep(0.5)

            # アームの衝突検知　場所の高さに応じて変更
            self._common_function.lowerArmWithCheckCollision(0.0, obj_info.grasp_point)                
            rospy.sleep(0.1)


            if place_h >= 0.3:
                # 少しアームを上げる
                current_joint_positions = self._librobotsub.getJointPositions()
                self._librobotaction.wholeBodyMoveToJointPositions(
                    [
                        "arm_lift_joint"   
                    ],
                    [
                        current_joint_positions["arm_lift_joint"] + 0.05
                    ]
                )

            self._librobotaction.gripperApplyForce(0.5)


        #==================================================

        # 正面から把持

        #==================================================
        elif obj_info.grasp_point == GP_GRASP_FRONT:
            self._librobotaction.wholeBodyMoveEndEffectorPose(
                95.0,
                20.0,
                "hand_palm_link",
                True,
                float(obj_info.pose.position.x) + offset_x,
                float(obj_info.pose.position.y) + offset_y,
                float(obj_info.pose.position.z) + offset_z,
                60.0 * G_DEG2RAD,
                -90.0 * G_DEG2RAD,
                90.0 * G_DEG2RAD,
                "base_link"
            )
            rospy.sleep(0.5)

            self._librobotaction.wholeBodyMoveEndEffectorPose(
                80.0,
                20.0,
                "hand_palm_link",
                True,
                0.0,
                0.0,
                -offset_x - 0.025,
                0.0,
                0.0,
                0.0,
                "hand_palm_link"
            )

            # 把持
            rospy.sleep(0.1)
            self._librobotaction.gripperApplyForce(0.5)


        # 少しアームを上げる
        current_joint_positions = self._librobotsub.getJointPositions()
        self._librobotpub.wholeBodyMoveToArmPositions(
            current_joint_positions["arm_lift_joint"] + lift_up_val,
            current_joint_positions["arm_flex_joint"] + arm_up_val * G_DEG2RAD,
            current_joint_positions["arm_roll_joint"],
            current_joint_positions["wrist_flex_joint"],
            current_joint_positions["wrist_roll_joint"],
            1.0
        )
        rospy.sleep(0.5)

        # 少しバックする
        for i in xrange(int(back_val * 1000.0)):
            self._librobotpub.omniBaseVel(
                -0.1,
                0.0,
                0.0
            )
            rospy.sleep(0.01)


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        grasp_obj_id = rospy.get_param(rospy.get_name() + "/grasp_obj/id", "")
        x = rospy.get_param(rospy.get_name() + "/grasp_obj/pose/x")
        y = rospy.get_param(rospy.get_name() + "/grasp_obj/pose/y")
        z = rospy.get_param(rospy.get_name() + "/grasp_obj/pose/z")

        print("==== getting now ====")
        print("grasp_obj_id = " + str(grasp_obj_id))
        print("x = " + str(x))
        print("y = " + str(y))
        print("z = " + str(z))

        grasp_obj_pose = Pose(
            Point(float(x), float(y), float(z)),
            Quaternion(0.0, 0.0, 0.0, 1.0)
        )

        #raw_input(">>> ")

        #==================================================

        # 把持点推定

        #==================================================
        shelf_flag = rospy.get_param(rospy.get_name() + "/shelf/flag", False)
        if shelf_flag is False:
            obj_info = self._common_function.getGraspPoint(
                10,
                grasp_obj_id,
                grasp_obj_pose,
                0.4,
                0.4,
                0.2
            )
            if obj_info is False:
                print("I couldn't get obj [obj_info is False]")

                # 把持点推定無しの場合
                obj_info_ = {"pose_x":0.0, "pose_y":0.0, "pose_z":0.0}
                obj_info_["pose_x"] = x
                obj_info_["pose_y"] = y
                obj_info_["pose_z"] = z

                #==================================================

                # 物体把持

                #==================================================
                self.graspObj(
                    obj_info_,
                    5.0,
                    0.1
                )

                return "normal"


            place_h = obj_info.pose.position.z - (obj_info.h / 2.0)
            place_grasp_point = -1

            # 把持点推定無しの場合
            #obj_info = {"pose_x":0.0, "pose_y":0.0, "pose_z":0.0}
            #obj_info["pose_x"] = x
            #obj_info["pose_y"] = y
            #obj_info["pose_z"] = z

            #==================================================

            # 物体把持

            #==================================================
            """
            self.graspObj(
                obj_info,
                place_h,
                place_grasp_point,
                45.0,
                0.15
            )
            """
            self.graspObjWithGetGraspPoint(
                obj_info,
                place_h,
                place_grasp_point,
                15.0,
                0.1
            )

        # shelfの場合
        else:
            obj_info = self._common_function.getGraspPoint(
                10,
                grasp_obj_id,
                grasp_obj_pose,
                0.4,
                0.4,
                0.2
            )
            if obj_info is False:
                print("I couldn't get obj [obj_info is False]")

                # 把持点推定無しの場合
                obj_info_ = {"pose_x":0.0, "pose_y":0.0, "pose_z":0.0}
                obj_info_["pose_x"] = x
                obj_info_["pose_y"] = y
                obj_info_["pose_z"] = z

                #==================================================

                # 物体把持

                #==================================================
                self.graspObj(
                    obj_info_,
                    5.0,
                    0.1
                )

                return "normal"


            place_h = obj_info.pose.position.z - (obj_info.h / 2.0)
            place_grasp_point = -1

            # 把持点推定無しの場合
            #obj_info = {"pose_x":0.0, "pose_y":0.0, "pose_z":0.0}
            #obj_info["pose_x"] = x
            #obj_info["pose_y"] = y
            #obj_info["pose_z"] = z

            #==================================================

            # 物体把持

            #==================================================
            """
            self.graspObj(
                obj_info,
                place_h,
                place_grasp_point,
                45.0,
                0.15
            )
            """
            self.graspObjWithGetGraspPoint(
                obj_info,
                place_h,
                GP_GRASP_TOP,
                0.0,
                0.1,
                lift_up_val = 0.01
            )
            


        #==================================================

        # 把持検証

        #==================================================
        if self._common_function.checkGrasp():
            print("I could get obj !!!")

            return "normal"

        else:
            print("I couldn't get obj [self._common_function.checkGrasp() is False]")

            #########################################################################
            # TODO オペレーションポイントへ移動

            #self._librobotaction.wholeBodyMoveToGo()
            self._librobotaction.gripperCommand(1.2)

            self._librobotnav.goPlace(
                GP_OPERATION_POINT,
                0.0,
                [
                    "arm_lift_joint",
                    "arm_flex_joint",
                    "arm_roll_joint",
                    "wrist_flex_joint",
                    "wrist_roll_joint",
                    "head_pan_joint",
                    "head_tilt_joint"
                ],
                [
                    0.0,
                    0.0,
                    -90.0, 
                    -90.0,
                    0.0,
                    0.0,
                    0.0
                ]
            )

            #########################################################################

            return "move"


        #### test !! ####
        #return "normal"
        #################




#==================================================

## @class オーダーされたものを渡す

#==================================================
class GiveObject(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        librobotpub,
        librobotsub,
        librobotaction,
        librobotspeech,
        librobotnav,
        libsub,
        libtf,
        libutil,
        libmap
    ):
        smach.State.__init__(self, outcomes = ["normal", "end", "except", "loop"])

        self._libsound = libsound
        self._librobotpub = librobotpub
        self._librobotsub = librobotsub
        self._librobotaction = librobotaction
        self._librobotspeech = librobotspeech
        self._librobotnav = librobotnav
        self._libsub = libsub
        self._libtf = libtf
        self._libutil = libutil
        self._libmap = libmap

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = self._init_ros_time

        self._robot_descriptor = self._libutil.getRobotDescriptor()

        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return



    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):

        count_get_obj = rospy.get_param(rospy.get_name() + "/count_get_obj", 0)     # 取得した物体のカウント



        #########################################################################
        # TODO オペレーションポイントへ移動

        self._librobotnav.goPlace(
            GP_OPERATION_POINT,
            0.0,
            [
                "arm_lift_joint",
                "arm_flex_joint",
                "arm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
                "head_pan_joint",
                "head_tilt_joint"
            ],
            [
                0.0,
                0.0,
                -90.0, 
                -90.0,
                0.0,
                0.0,
                0.0
            ]
        )

        #########################################################################



        #########################################################################
        # TODO オブジェクトを渡す姿勢

        self._librobotpub.HSRDisplay(0)
        txt = ["", "Operator, please comming front of me."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        
        cmd = "rosrun hma_common_pkg detect_speaker_once_node.py"
        call(cmd.split(" "))

        # 顔を上げる
        self._librobotaction.wholeBodyMoveToJointPositions(
            [
                "head_tilt_joint"
            ],
            [
                29.0 * G_DEG2RAD
            ]
        )
        rospy.sleep(1.0)

        # 物体が見つからなかった場合
        """
        find_obj_flag = rospy.get_param(rospy.get_name() + "/find/object/flag", False)
        if find_obj_flag is False:
            txt = ["", "Sorry, I couldn't find the object."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

            return "normal"
        """

        txt = ["", "Here you are."]
        self._librobotspeech.talkRequestNonSync(txt[GP_LANG], GP_LANG)

        self._librobotaction.wholeBodyMoveToJointPositions(
            [
                "arm_lift_joint",
                "arm_flex_joint",
                "arm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",    
            ],
            [
                0.4,
                -45.0 * G_DEG2RAD,
                0.0,
                -45.0 * G_DEG2RAD,
                0.0,
            ]
        )

        txt = ["", "I will release."]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        txt = ["", "3"]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        txt = ["", "2"]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)
        txt = ["", "1"]
        self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

        self._librobotaction.gripperCommand(1.2)

        rospy.sleep(1.0)
        self._librobotaction.wholeBodyMoveToGo()

        #########################################################################

        count_get_obj += 1
        rospy.set_param(rospy.get_name() + "/count_get_obj", count_get_obj) # 認識した物体のカウント

        if count_get_obj > 2:

            rospy.sleep(1.0)

            txt = ["テスト完了．", "test completed."]
            self._librobotspeech.talkRequestSync(txt[GP_LANG], GP_LANG)

            return "end"

        else:

            return "normal"






#==================================================

## @class 正常終了ステート

#==================================================
class NormalEnd(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["normal"])


        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "normal"




#==================================================

## @class 例外終了ステート

#==================================================
class Except(
    smach.State
):
    #==================================================

    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["except"])


        return


    #==================================================

    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "except"




#==================================================

# メイン

#==================================================
if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])


    libutil = LibUtil()
    libmysql = LibMySQL(libutil, GP_MYSQL_HOST, GP_MYSQL_USR, GP_MYSQL_PASSWORD, GP_MYSQL_DB)
    libsound = LibSound(libutil)
    libopencv = LibOpenCV(libutil)
    libtf = LibTF(libutil)
    libsub = LibSub(libutil, libopencv)
    libaction = LibAction(libutil, libopencv)
    librobotpub = LibRobotPub(libutil, libopencv)
    librobotsub = LibRobotSub(libutil, libopencv)
    librobotaction = LibRobotAction(libutil, libopencv)
    librobotspeech = LibRobotSpeech(libutil, libsound)
    librobotnav = LibRobotNav(libutil, libmysql, libtf, librobotpub, librobotaction, librobotspeech)
    librobotimp = LibRobotImp(libutil, librobotaction)
    libmap = LibMap(libutil)

    ###  from gpsr  ###
    libspeech = LibSpeech()

    robot_descriptor = libutil.getRobotDescriptor()

    common_function = CommonFunction(
        # 共通ライブラリクラスのインスタンス
        libutil, libmysql, libsound, libopencv, libtf, libsub, libaction,
        # ロボットライブラリクラスのインスタンス
        librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, librobotimp
    )

    rospy.loginfo("[" + rospy.get_name() + "]: Task start")
    rospy.loginfo("[" + rospy.get_name() + "]: Please input first state name")


    start_state = raw_input(">>> ")
    if not start_state:
        start_state = "NormalStart"


    #==================================================

    # ステートマシンの宣言

    #==================================================
    ssm = smach.StateMachine(outcomes = ["exit"])
    """
    ask_favorite
    recognize_age
    open_door

    introduce_guest_1
    sit_guest_1

    go_to_entrance

    ask_favorite
    recognize_age
    open_door

    introduce_guest_2
    sit_guest_2
    """
    with ssm:
        ######################
        # Training Phase
        ######################
        smach.StateMachine.add(
            "Init",
            Init(),
            transitions = {
                "normal":"WaitStart",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "WaitStart",
            WaitStart(libsound, librobotpub, librobotsub, librobotaction, librobotspeech),
            transitions = {
                "normal":start_state,
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "NormalStart",
            NormalStart(),
            transitions = {
                "normal":"OpenDoor",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "OpenDoor",
            OpenDoor(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub),
            transitions = {
                "normal":"GoToStartPoint",
                "except":"Except",
                "loop":"OpenDoor"
            }
        )
        smach.StateMachine.add(
            "GoToStartPoint",
            GoToStartPoint(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libmap),
            transitions = {
                "train":"FollowMe",
                "test":"TakeOrder",
                "except":"Except",
                "loop":"GoToStartPoint"
            }
        )
        smach.StateMachine.add(
            "FollowMe",
            FollowMe(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libmap),
            transitions = {
                "normal":"RecObject",
                "end":"GoToStartPoint",
                "except":"Except",
                "loop":"FollowMe"
            }
        )
        smach.StateMachine.add(
            "RecObject",
            RecObject(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libspeech),
            transitions = {
                "normal":"FollowMe",
                "except":"Except",
                "loop":"RecObject"
            }
        )
        ######################
        # Test Phase
        ######################
        smach.StateMachine.add(
            "TakeOrder",
            TakeOrder(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libspeech),
            transitions = {
                "normal":"GoToObject",
                "except":"Except",
                "loop":"TakeOrder"
            }
        )
        smach.StateMachine.add(
            "GoToObject",
            GoToObject(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libspeech, common_function),
            transitions = {
                "normal":"TakeObject",
                "end":"TakeOrder",
                "except":"Except",
                "loop":"GoToObject"
            }
        )
        smach.StateMachine.add(
            "TakeObject",
            TakeObject(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libspeech, common_function),
            transitions = {
                "normal":"GiveObject",
                "move":"TakeOrder",
                "except":"Except",
                "loop":"TakeObject"
            }
        )
        smach.StateMachine.add(
            "GiveObject",
            GiveObject(libsound, librobotpub, librobotsub, librobotaction, librobotspeech, librobotnav, libsub, libtf, libutil, libspeech),
            transitions = {
                "normal":"TakeOrder",
                "end":"NormalEnd",
                "except":"Except",
                "loop":"GiveObject"
            }
        )
        smach.StateMachine.add(
            "NormalEnd",
            NormalEnd(),
            transitions = {
                "normal":"exit"
            }
        )
        smach.StateMachine.add(
            "Except",
            Except(),
            transitions = {
                "except":"exit"
            }
        )

    sris = smach_ros.IntrospectionServer("ssm", ssm, "/SM_ROOT")
    sris.start()


    while not rospy.is_shutdown():
        ssm.execute()
