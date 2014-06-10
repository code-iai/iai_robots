;;;
;;; SCRIPTING FUNCTIONALITY USED TO CONVERT INFORMATION 
;;; PUBLISHED ON /pressure/[lr]_gripper_motor_info INTO
;;; A YAML REPRESENTATION OF THE PR2 GRIPPER PRESSURE
;;; SENSORS.
;;; 
;;; NOTE: INPUT-DATA EXPECTED TO BE ROS-MSGS FROM 
;;; PACKAGE FINTERTIP_PRESSURE
;;;

(defun write-string-to-file (filename string-datum)
  (with-open-file (s filename :direction :output :if-exists :supersede)
    (format s string-datum)))

(defun yamlify-gripper-sensor-msgs (l-gripper-msg r-gripper-msg)
  (concatenate
   'string
   (yamlify-gripper l-gripper-msg "l_gripper" 0 t)
   (yamlify-gripper r-gripper-msg "r_gripper" 0 nil)))

(defun yamlify-gripper (gripper gripper-name indents newline-p)
  (with-fields (sensor) gripper
    (concatenate 
     'string
     (yamlify-name (concatenate 'string gripper-name "_sensors") indents t)
     (yamlify-finger (aref sensor 0) (finger-name gripper-name t) (+ indents 2) t)
     (yamlify-finger (aref sensor 1) (finger-name gripper-name nil) (+ indents 2) newline-p))))
      

(defun yamlify-finger (finger finger-name indents newline-p)
  (let ((result (yamlify-name finger-name indents t)))
    (with-fields (center halfside1 halfside2 force_per_unit frame_id) finger
      (loop for i from 0 to (- (length center) 1) do
        (setf result (concatenate 'string result
                                  (yamlify-sensor finger-name i frame_id (aref center i) (aref halfside1 i) (aref halfside2 i) (aref force_per_unit i) (+ indents 2) 
                                                  (or (< i (- (length center) 1)) newline-p))))))
    result))

(defun yamlify-sensor (finger-name index frame-id center halfside1 halfside2 force-per-unit indents newline-p)
  (concatenate 
   'string
   (yamlify-name (sensor-name finger-name index) indents t)
   (yamlify-element "finger_index" index (+ 2 indents) t)
   (yamlify-element "frame_id" frame-id (+ 2 indents) t)
   (yamlify-3d-vector center "center" (+ 2 indents) t)
   (yamlify-3d-vector halfside1 "halfside1" (+ 2 indents) t)
   (yamlify-3d-vector halfside2 "halfside2" (+ 2 indents) t)
   (yamlify-element "force_per_unit" force-per-unit (+ 2 indents) newline-p)))

(defun yamlify-3d-vector (vec name indents newline-p)
  (with-fields (x y z) vec
    (concatenate
     'string
     (yamlify-name name indents t)
     (yamlify-element "x" x (+ indents 2) t)
     (yamlify-element "y" y (+ indents 2) t)
     (yamlify-element "z" z (+ indents 2) newline-p))))

(defun yamlify-element (name value indents newline-p)
  (concatenate
   'string
   (yamlify-name name indents nil)
   (yamlify-value value newline-p)))

(defun yamlify-name (name indents newline-p)
  (concatenate
   'string
   (indent indents)
   name ":"
   (if newline-p
       (newline-string)
       " ")))

(defun yamlify-value (value newline-p)
  (concatenate
   'string
   (if (typep value 'string)
       value
       (write-to-string value))
   (if newline-p
       (newline-string)
       "")))

(defun indent (indents)
  (if (> indents 0)
      (apply #'concatenate 'string (loop for n from 1 to indents collect " "))
      ""))

(defun newline-string ()
  (list #\newline))

(defun finger-name (gripper-name left-p)
  (concatenate 'string gripper-name (if left-p "_l_" "_r_") "finger_sensors"))

(defun sensor-name (finger-name index)
  (concatenate 'string finger-name "_sensor" (write-to-string index)))