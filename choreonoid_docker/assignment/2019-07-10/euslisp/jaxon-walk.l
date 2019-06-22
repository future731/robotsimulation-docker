(load "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
(load "package://hrpsys_choreonoid_tutorials/euslisp/action_and_perception/jvrc-standup.l")

(defun init ()
  (jaxon_jvrc-init)
  (objects (list *robot*)))

(defun look-down ()
  (sync-with-ri)
  (send *robot* :look-at-target #f(1000 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000))

(defun sync-with-ri ()
  (print (send *robot* :angle-vector (send *ri* :state :potentio-vector))))

(defun squat (squat-height)
  (send *robot* :legs :move-end-pos (float-vector 0 0 squat-height))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000))

(defun larm-trans-ik (move-absolute-coords)
  (let (l)
    (sync-with-ri)
    (send *robot* :larm :inverse-kinematics move-absolute-coords
      :translation-axis t
      :rotation-axis t
      :revert-if-fail nil
      :debug-view nil)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    ))

(defun larm-trans-ik-with-abs-pos (abs-pos-from-base)
  (larm-trans-ik (progn (setq l (copy-object (send *robot* :larm :end-coords))) (send l :locate abs-pos-from-base :world)))
  )

(defun larm-trans (move-relative-3axis)
  (let (l)
    (sync-with-ri)
    (setq l (copy-object (send *robot* :larm :end-coords)))
    (send l :translate move-relative-3axis)
    (send *robot* :larm :inverse-kinematics l
      :translation-axis t
      :rotation-axis t
      :revert-if-fail nil
      :debug-view nil)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    ))

(defun larm-rot (move-relative-3axis)
  (let (l)
    (sync-with-ri)
    (setq l (copy-object (send *robot* :larm :end-coords)))
    (send l :rotate (elt move-relative-3axis 0) :x :local)
    (send l :rotate (elt move-relative-3axis 1) :y :local)
    (send l :rotate (elt move-relative-3axis 2) :z :local)
    (send *robot* :larm :inverse-kinematics l
      :translation-axis t
      :rotation-axis t
      :revert-if-fail nil
      :debug-view nil)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    ))


(defun my-initialize ()
  (init)
  (send *ri* :go-pos 3 0 0)
  (look-down)
  (squat 300)
  (larm-trans #f(0 100 0))
  (larm-trans #f(0 0 100))
  (send *ri* :go-pos 0 -0.5 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0 -0.5 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0 -0.2 0)
  (send *ri* :go-pos 0.2 0 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos 0.2 0 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos 0.2 0 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos 0.2 0 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos 0.2 0 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos 0 0.4 0)
  (send *ri* :go-pos 0 -0.3 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0 -0.3 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0 -0.3 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0 -0.3 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos -0.1 0 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0.3 0 0)
  (send *ri* :go-pos 0 0.1 0)
  (send *ri* :go-pos 0 -0.1 0)
  (send *ri* :go-pos 0 -0.1 0)
  (send *ri* :go-pos 0.5 0 0)
  (send *ri* :go-pos 0.2 0 0)
  (send *ri* :go-pos -2.0 0 0)
  (send *ri* :go-pos 0 4.0 0)
  (send *ri* :go-pos 5.0 0 0)
  (send *ri* :go-pos 2.0 0 0)
  (send *ri* :go-pos 0 0.2 0)
  (send *ri* :go-pos 0 -1.0 0)
  (squat -300)
  )
