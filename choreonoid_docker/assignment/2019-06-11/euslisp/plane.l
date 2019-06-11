#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus "plane-detector")

(defun point-to-vector (point)
  (let (tf pos pos-body)
      (setq tf (send *tr* :lookup-transform "BODY" "left_camera_optical_frame" (ros::time)))
      (setq pos (scale 1000 (float-vector (send point :x) (send point :y) (send point :z)))) ;; [mm]
      (setq pos-body (send tf :transform-vector pos))
      (scale 0.001 ; [mm] -> [m]
             pos-body)))

;; area is calculated by 1/2 * | \sum_{i = 1}^n (\mathbf{xi} x \mathbf{xi+1}) |
(defun points-to-area (points)
  (setq point-pre (car points))
  (setq area #f(0 0 0))
  (dolist (point (cdr points))
    (setq area (v+ area (v* point-pre point)))
    (setq point-pre point)
    )
  (setq area (v+ area (v* (car (last points)) (car points))))
  (setq area (/ (norm area) 2.0))
  area
  )

(defclass cb-class
  :super propertied-object
  :slots (marker-id))

(defmethod cb-class
  (:init ()
         (setq marker-id 0)
         (ros::subscribe "/multi_plane_estimate/output_polygon" jsk_recognition_msgs::PolygonArray #'send self :update-cb)
         )
  (:update-cb (msg)
    (setq header (send msg :header))
    (setq polygons (send msg :polygons))
    (print "update-cb start")
    ;; calculate area for each polygons
    (send *tr* :wait-for-transform "BODY" "left_camera_optical_frame" (ros::time-now) 3)
    (dolist (polygon polygons)
      (setq points (map cons #'point-to-vector (send polygon :polygon :points)))
      (setq max-x (reduce #'max (map cons #'(lambda (e) (elt e 0)) points)))
      (setq min-x (reduce #'min (map cons #'(lambda (e) (elt e 0)) points)))
      (setq max-y (reduce #'max (map cons #'(lambda (e) (elt e 1)) points)))
      (setq min-y (reduce #'min (map cons #'(lambda (e) (elt e 1)) points)))
      (setq max-z (reduce #'max (map cons #'(lambda (e) (elt e 2)) points)))
      (setq min-z (reduce #'min (map cons #'(lambda (e) (elt e 2)) points)))
      (setq center (scale (/ 1.0 (length points)) (reduce #'v+ points)))
      (setq area (points-to-area points))
      (if (and (> 1.0 (abs min-x)) (> (abs max-x) 2.0) (> area 1))
        ;; true
        (format t "[this is target]: area: ~7,3f max-x: ~7,3f min-x: ~7,3f max-y: ~7,3f min-y: ~7,3f max-z: ~7,3f min-z: ~7,3f center: ~A~%"
                  area max-x min-x max-y min-y max-z min-z center)
        ;; else
        (format t "area: ~7,3f max-x: ~7,3f min-x: ~7,3f max-y: ~7,3f min-y: ~7,3f max-z: ~7,3f min-z: ~7,3f center: ~A~%"
                  area max-x min-x max-y min-y max-z min-z center)
        )
      )
    )
  )

(setq *tr* (instance ros::transform-listener :init))

(setq sub (instance cb-class :init))
(while (ros::ok) (ros::spin-once) (ros::sleep))
