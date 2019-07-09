#!/usr/bin/env roseus
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus "assignment2")

(load "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(load "package://hrpsys_choreonoid_tutorials/euslisp/action_and_perception/jvrc-standup.l")
(setq *tr* (instance ros::transform-listener :init))
(unix::usleep (* 2 1000 1000))
(send *tr* :wait-for-transform "BODY" "left_camera_optical_frame" (ros::time-now) 2)


(defun init ()
  (jaxon_jvrc-init)
  (objects (list *robot*)))

(defun look-down (x-from-base)
  (sync-with-ri)
  (send *robot* :look-at-target (float-vector x-from-base 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000))

(defun sync-with-ri ()
  (print (send *robot* :angle-vector (send *ri* :state :potentio-vector))))

(defun squat (squat-height)
  (send *robot* :legs :move-end-pos (float-vector 0 0 squat-height))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000))

(defun trans-ik (move-absolute-coords tm (limb :larm))
  (let (l)
    (sync-with-ri)
    (format t "ik target coords: ~A~%" move-absolute-coords)
    (setq *last-ik-target-coords* move-absolute-coords)
    (send *robot* limb :inverse-kinematics move-absolute-coords
      :translation-axis t
      :rotation-axis t
      :revert-if-fail t
      :debug-view nil)
    (send *ri* :angle-vector (send *robot* :angle-vector) tm)
    ))

(defun trans-ik-with-abs-pos (abs-pos-from-base tm limb)
  (trans-ik (progn (setq l (copy-object (send *robot* limb :end-coords))) (send l :locate abs-pos-from-base :world)) tm limb)
  )

(defun trans-ik-with-abs-pos-and-ypr (abs-pos-from-base abs-ypr-from-base tm limb)
  (trans-ik (progn (setq l (copy-object (send *robot* limb :end-coords)))
                        (send l :locate abs-pos-from-base :world)
                        (send l :orient (elt abs-ypr-from-base 0) :z :world)
                        (send l :rotate (elt abs-ypr-from-base 1) :y :local)
                        (send l :rotate (elt abs-ypr-from-base 2) :x :local)) tm limb)
  )

(defun rel-trans (move-relative-3axis limb)
  (let (l)
    (sync-with-ri)
    (setq l (copy-object (send *robot* limb :end-coords)))
    (send l :translate move-relative-3axis)
    (send *robot* limb :inverse-kinematics l
      :translation-axis t
      :rotation-axis t
      :revert-if-fail t
      :debug-view nil)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    ))

(defun rel-rot (move-relative-3axis limb)
  (let (l)
    (sync-with-ri)
    (setq l (copy-object (send *robot* limb :end-coords)))
    (send l :rotate (elt move-relative-3axis 0) :x :local)
    (send l :rotate (elt move-relative-3axis 1) :y :local)
    (send l :rotate (elt move-relative-3axis 2) :z :local)
    (send *robot* limb :inverse-kinematics l
      :translation-axis t
      :rotation-axis t
      :revert-if-fail t
      :debug-view nil)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    ))

(defun transform-ros-point (point)
  (let (tf pos pos-body)
    (send *tr* :wait-for-transform "BODY" "left_camera_optical_frame" (ros::time-now) 1)
    (setq tf (send *tr* :lookup-transform "BODY" "left_camera_optical_frame" (ros::time)))
    (setq pos (scale 1000 (float-vector (send point :x) (send point :y) (send point :z)))) ;; [mm]
    (setq pos-body (send tf :transform-vector pos))
    (scale 0.001 pos-body) ;; [mm] -> [m]
    )
  )

(defun ik-white (ypr-float-vector pos-offset)
  (setq *transformed-point-vector* (get-white-point))
  (ik-with-offset ypr-float-vector pos-offset :larm)
  (send *ri* :wait-interpolation)
  (ik-with-offset ypr-float-vector #f(0 0 0) :larm)
  )
(defun ik-red (ypr-float-vector pos-offset)
  (setq *transformed-point-vector* (get-white-point))
  (ik-with-offset ypr-float-vector pos-offset :rarm)
  (send *ri* :wait-interpolation)
  (ik-with-offset ypr-float-vector #f(0 0 0) :rarm)
  )

(defun get-white-point ()
  (setq *msgs* (one-shot-subscribe "/white_box/HSI_color_filter/cluster_decomposer/centroid_pose_array" geometry_msgs::PoseArray))
  (setq min-distance 10)
  (dolist (msg (send *msgs* :poses))
    (setq point (send msg :position))
    (when (< (norm (float-vector (send point :x)
                                 (send point :y)
                                 (send point :z))) min-distance)
      (setq *msg* msg)
      (setq min-distance (norm (float-vector (send point :x)
                                             (send point :y)
                                             (send point :z))))))
  (setq *point* (send *msg* :position))
  (format t "~A ~A ~A~%" (send *point* :x)(send *point* :y)(send *point* :z))
  (scale 1000 (transform-ros-point *point*)) ;; [mm]
  )
(defun get-red-point ()
  (setq *msgs* (one-shot-subscribe "/red_box/HSI_color_filter/cluster_decomposer/centroid_pose_array" geometry_msgs::PoseArray))
  (setq min-distance 10)
  (dolist (msg (send *msgs* :poses))
    (setq point (send msg :position))
    (when (< (norm (float-vector (send point :x)
                                 (send point :y)
                                 (send point :z))) min-distance)
      (setq *msg* msg)
      (setq min-distance (norm (float-vector (send point :x)
                                             (send point :y)
                                             (send point :z))))))
  (setq *point* (send *msg* :position))
  (format t "~A ~A ~A~%" (send *point* :x)(send *point* :y)(send *point* :z))
  (scale 1000 (transform-ros-point *point*)) ;; [mm]
  )

(defun ik-with-offset (ypr-float-vector pos-offset limb)
  (trans-ik-with-abs-pos-and-ypr (v+ *transformed-point-vector* pos-offset) ypr-float-vector 10000 limb)
  ;; (trans-ik-with-abs-pos *transformed-point-vector* 10000 :larm)
  )

(defun get-white ()
  (send *ri* :go-pos 4.5 -1.0 0)
  (look-down 800)
  (unix::usleep (* 2 1000 1000))
  (ik-white (float-vector 0 pi/2 0) #f(0 0 0))
  )

(defun get-white-on-table ()
  (send *ri* :go-pos 5.2 -1.2 0)
  (look-down 800)
  (unix::usleep (* 2 1000 1000))
  (ik-white (float-vector 0 pi/2 0) #f(0 0 300))
  )

(defun get-red-on-table ()
  (send *ri* :go-pos 5.2 -1.2 0)
  (look-down 800)
  (unix::usleep (* 2 1000 1000))
  (ik-red (float-vector 0 pi/2 0) #f(0 0 300))
  )

(defun my-initialize ()
  (init)
  ;;(send *ri* :go-pos 3.6 1.0 0)
  (look-down 800)
  ;; (squat 420)
  (unix::usleep (* 2 1000 1000))
  (ik-white (float-vector 0 pi/2 0))

  ;; (larm-trans #f(0 100 0))
  ;; (larm-trans #f(0 0 100))
  ;; (squat -300)
  )
