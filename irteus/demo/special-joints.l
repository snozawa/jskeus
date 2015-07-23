;;;;;;;;;;;;;;;;
;; Example for special joints
;;;;;;;;;;;;;;;;

(load "sample-multidof-arm-model.l")
(defmethod sample-multidof-arm-robot
  (:interlocking-joint-pairs
   ()
   "Interlocking joint pairs."
   (mapcar #'(lambda (x y)
              (cons x y))
           (butlast (send self :joint-list)) (cdr (send self :joint-list)))
   )
  )

(defun test-interlocking-joint-arm (&key (debug-view :no-message))
  "Test code for interlocking joint.
   All joints of sample-multidof-arm-robot are interlocking joint (same joint values)."
  (setq *robot* (instance sample-multidof-arm-robot :init :axis-order (list :y :y :y)))
  (objects (list *robot*))
  (let ((ret))
    (dotimes (i 25)
      (send *robot* :inverse-kinematics
            (send (send *robot* :rarm :end-coords :copy-worldcoords)
                  :translate (float-vector (cond ((< i 5) 20) ((< i 15) -20) ((< i 25) 20) (t -20)) 0 0))
            :move-target (send *robot* :rarm :end-coords)
            :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
            :rotation-axis nil :translation-axis :z
            :additional-jacobi
            (list #'(lambda (ll)
                      (send *robot* :calc-jacobian-for-interlocking-joints ll)))
            :additional-vel (list #'(lambda (ll) (send *robot* :calc-vel-for-interlocking-joints ll)))
            :move-joints-hook
            #'(lambda ()
                (send *robot* :set-midpoint-for-interlocking-joints)
                nil)
            :debug-view debug-view)
      (send *irtviewer* :draw-objects)
      (format t ";; joint angles = ~A => interlocking? ~A~%"
              (send *robot* :angle-vector) (send *robot* :check-interlocking-joint-angle-validity))
      (push (send *robot* :check-interlocking-joint-angle-validity) ret)
      )
    (every #'identity (reverse ret))
    ))

(defclass sample-legged-robot-with-interlocking-joints
  :super robot-model
  :slots ()
  )

(defmethod sample-legged-robot-with-interlocking-joints
  (:init
   ()
   (prog1
       (send-super :init :name "sample-legged-robot-with-interlocking-joints")
     ;; 1. Make simple link robot model
     ;;    make link
     (let* ((l0 (send self :make-link :root-link :green :depth 200 :width 200 :height 500 :trans 250 :weight 20000)))
       (setq rleg (send self :make-leg-links :rleg l0))
       (setq lleg (send self :make-leg-links :lleg l0))
       (send (car rleg) :translate (float-vector 0 -100 0))
       (send (car lleg) :translate (float-vector 0 100 0))
       (setq rleg-end-coords (make-cascoords :coords (send (send (car (last rleg)) :copy-worldcoords) :translate (float-vector 0 0 -100))))
       (setq lleg-end-coords (make-cascoords :coords (send (send (car (last lleg)) :copy-worldcoords) :translate (float-vector 0 0 -100))))
       (send (car (last rleg)) :assoc rleg-end-coords)
       (send (car (last lleg)) :assoc lleg-end-coords)
       (send self :assoc l0)
       ;;    make joint
       (setq links (append (list l0) rleg lleg))
       (setq joint-list (append (send self :make-leg-joints l0 rleg :rleg) (send self :make-leg-joints l0 lleg :lleg)))
       ;; 3. Call init-ending
       (send self :init-ending)
       ))
   )
  (:make-leg-links
   (l/r root-link)
   (let ((l0 (send self :make-link (format nil "~A-link0" l/r) :yellow))
         (l1 (send self :make-link (format nil "~A-link1" l/r) :red))
         (l2 (send self :make-link (format nil "~A-link2" l/r) :blue))
         (l3 (send self :make-link (format nil "~A-link3" l/r) :green))
         (l4 (send self :make-link (format nil "~A-link4" l/r) :yellow))
         (l5 (send self :make-link (format nil "~A-link5" l/r) :red))
         (l6 (send self :make-link (format nil "~A-link6" l/r) :blue)))
     (mapcar #'(lambda (pl cl)
                 (send pl :assoc cl))
             (list root-link l0 l1 l2 l3 l4 l5)
             (list l0 l1 l2 l3 l4 l5 l6))
     (mapcar #'(lambda (l)
                 (send l :translate (float-vector 0 0 -100)))
             (list l1 l2 l3 l4 l5 l6))
     (list l0 l1 l2 l3 l4 l5 l6)
     ))
  (:make-leg-joints
   (root-link leg-links l/r)
   (mapcar #'(lambda (pl cl ax)
               (instance rotational-joint :init
                         :parent-link pl :child-link cl
                         :axis ax :name (format nil "~A-joint~d" l/r (position cl leg-links))))
           (append (list root-link) (butlast leg-links))
           leg-links
           (list :z :x :y :y :y :y :x))
   )
  (:make-link
   (name color
    &key (depth 50) (width 50) (height 100) (trans (* -0.5 height))
         (weight 1000))
   (let ((b0 (make-cube depth width height)))
     (send b0 :translate (float-vector 0 0 trans))
     (send b0 :set-color color)
     (let ((ln (instance bodyset-link :init (make-cascoords)
                         :bodies (list b0) :name name
                         :centroid (send b0 :centroid)
                         :weight weight)))
       ln)))
  (:interlocking-joint-pairs
   ()
   (list (cons (send self :joint ":rleg-joint3") (send self :joint ":rleg-joint4"))
         (cons (send self :joint ":lleg-joint3") (send self :joint ":lleg-joint4")))
   )
  )

(defun test-sample-legged-robot-with-interlocking-joints
  (&key (cog-null-space nil))
  "Test code for legged robot with interlocking joints like KOJIRO robot.
   Knee joints are interlocking joint (same joint values)."
  (setq *robot* (instance sample-legged-robot-with-interlocking-joints :init))
  (send *robot* :rleg :angle-vector #f(0.0 0.0 -40.0 40.0 40.0 -40.0 0.0))
  (send *robot* :lleg :angle-vector #f(0.0 0.0 -40.0 40.0 40.0 -40.0 0.0))
  (objects (list *robot*))
  (let ((ret))
    (dotimes (i 16)
      (send *robot* :move-centroid-on-foot
            :both '(:rleg :lleg)
            :cog-null-space cog-null-space
            ;; Move COG for testing
            :target-centroid-pos (float-vector (* (sin (* 22.5 (deg2rad i))) 80) 0 0)
            :additional-jacobi
            (list #'(lambda (ll)
                      (send *robot* :calc-jacobian-for-interlocking-joints ll)))
            :additional-vel (list #'(lambda (ll) (send *robot* :calc-vel-for-interlocking-joints ll)))
            :move-joints-hook
            #'(lambda ()
                (send *robot* :set-midpoint-for-interlocking-joints)
                nil))
      (send *irtviewer* :draw-objects)
      (format t ";; joint angles = ~A => interlocking? ~A~%"
              (mapcar #'(lambda (jl) (send-all jl :joint-angle)) (send *robot* :interlocking-joint-pairs))
              (send *robot* :check-interlocking-joint-angle-validity))
      (push (send *robot* :check-interlocking-joint-angle-validity) ret)
      )
    (every #'identity (reverse ret))
    ))
(warn "(test-interlocking-joint-arm)~%")
(warn "(test-sample-legged-robot-with-interlocking-joints)~%")


(defclass sample-base-closed-link-robot
  :super robot-model
  :slots ()
  )

(defmethod sample-base-closed-link-robot
  (:init
   ()
   (prog1
       (send-super :init :name "sample-base-closed-link-robot")
     (let* ((l0 (send self :make-link :root-link :green :width 100 :depth 20 :height 20 :trans (float-vector 0 0 0)))
            (l1 (send self :make-link :link1 :red))
            (l2 (send self :make-link :link2 :blue :width 100 :depth 20 :height 20 :trans (float-vector 0 -50 0)))
            (l3 (send self :make-link :link3 :red :trans (float-vector 0 0 -50)))
            (l4 (send self :make-link :link4 :green))
            (l5 (send self :make-link :link5 :blue))
            (l6 (send self :make-link :link6 :red)))
       (send l1 :translate (float-vector 0 50 0))
       (send l2 :translate (float-vector 0 50 100))
       (send l3 :translate (float-vector 0 -50 100))
       (send l4 :translate (float-vector 0 0 100))
       (send l5 :translate (float-vector 0 0 200))
       (send l6 :translate (float-vector 0 0 300))
       (let ((c (make-cascoords :coords (send (send l3 :copy-worldcoords) :translate (float-vector 0 0 -100))
                                :name "l3-end-coords")))
         (send l3 :assoc c)
         (push c end-coords-list))
       (let ((c0 (make-cascoords :coords (send (send l0 :copy-worldcoords) :translate (float-vector 0 -50 0))
                                 :name "l0-end-coords"))
             (c1 (make-cascoords :coords (send (send l0 :copy-worldcoords) :translate (float-vector 0 -50 0))
                                 :name "l3-end-coords"))
             (c2 (make-cascoords :coords (send (send l6 :copy-worldcoords) :translate (float-vector 0 0 100))
                                 :name "l6-end-coords")))
         (send l0 :assoc c0)
         (send l3 :assoc c1)
         (send l6 :assoc c2)
         (push c0 end-coords-list)
         (push c1 end-coords-list)
         (push c2 end-coords-list))
       (mapcar #'(lambda (pl cl)
                   (send pl :assoc cl))
               (list self l0 l1 l2 l2 l4 l5)
               (list l0   l1 l2 l3 l4 l5 l6))
       (setq joint-list
             (mapcar #'(lambda (pl cl ax)
                         (instance rotational-joint :init
                                   :axis ax :name (format nil ":joint~A" (string-left-trim "link" (string-downcase (send cl :name))))
                                   :parent-link pl :child-link cl))
                     (list l0 l1 l2 l2 l4 l5)
                     (list l1 l2 l3 l4 l5 l6)
                     (list :x :x :x :z :y :x)))
       (setq links (list l0 l1 l2 l3 l4 l5 l6))
       (send self :init-ending)
       ))
   )
  (:make-link
   (name color
    &key (depth 20) (width 20) (height 100) (trans (float-vector 0 0 (* 0.5 height)))
         (weight 1000))
   (let ((b0 (make-cube depth width height)))
     (send b0 :translate trans)
     (send b0 :set-color color)
     (let ((ln (instance bodyset-link :init (make-cascoords)
                         :bodies (list b0) :name name
                         :centroid (send b0 :centroid)
                         :weight weight)))
       ln)))
  )

(defun test-sample-base-closed-link-robot
  ()
  (setq *robot* (instance sample-base-closed-link-robot :init))
  (objects (list *robot*))
  (let ((ret))
    (send *robot* :angle-vector #f(-30.0 -30.0 -30.0 0.0 0.0 0.0))
    (send *robot* :inverse-kinematics
          (send *robot* :end-coords "l0-end-coords")
          :link-list (send *robot* :link-list (send (send *robot* :end-coords "l3-end-coords") :parent))
          :move-target (send *robot* :end-coords "l3-end-coords")
          :rotation-axis :zy :translation-axis :x)
    (send *robot* :inverse-kinematics
          (list (send (send (send *robot* :end-coords "l6-end-coords") :copy-worldcoords) :translate (float-vector 40 40 -40))
                (send *robot* :end-coords "l0-end-coords"))
          :debug-view t
          :rotation-axis (list nil nil)
          :translation-axis (list t :x)
          :link-list (list (send *robot* :link-list (send (send *robot* :end-coords "l6-end-coords") :parent))
                           (send *robot* :link-list (send (send *robot* :end-coords "l3-end-coords") :parent)))
          :move-target (list (send *robot* :end-coords "l6-end-coords") (send *robot* :end-coords "l3-end-coords"))
          )
    ))
