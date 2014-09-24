; Auto-generated. Do not edit!


(cl:in-package multi_agent_planner_node-srv)


;//! \htmlinclude GetSwarmPlan-request.msg.html

(cl:defclass <GetSwarmPlan-request> (roslisp-msg-protocol:ros-message)
  ((planning_mode
    :reader planning_mode
    :initarg :planning_mode
    :type cl:fixnum
    :initform 0)
   (allocated_planning_time
    :reader allocated_planning_time
    :initarg :allocated_planning_time
    :type cl:float
    :initform 0.0)
   (swarm_start
    :reader swarm_start
    :initarg :swarm_start
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (swarm_goal
    :reader swarm_goal
    :initarg :swarm_goal
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (tolerance
    :reader tolerance
    :initarg :tolerance
    :type cl:float
    :initform 0.0)
   (initial_eps
    :reader initial_eps
    :initarg :initial_eps
    :type cl:float
    :initform 0.0)
   (final_eps
    :reader final_eps
    :initarg :final_eps
    :type cl:float
    :initform 0.0)
   (dec_eps
    :reader dec_eps
    :initarg :dec_eps
    :type cl:float
    :initform 0.0)
   (sbpl_planner
    :reader sbpl_planner
    :initarg :sbpl_planner
    :type cl:fixnum
    :initform 0)
   (planner_type
    :reader planner_type
    :initarg :planner_type
    :type cl:fixnum
    :initform 0)
   (meta_search_type
    :reader meta_search_type
    :initarg :meta_search_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetSwarmPlan-request (<GetSwarmPlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSwarmPlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSwarmPlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_planner_node-srv:<GetSwarmPlan-request> is deprecated: use multi_agent_planner_node-srv:GetSwarmPlan-request instead.")))

(cl:ensure-generic-function 'planning_mode-val :lambda-list '(m))
(cl:defmethod planning_mode-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:planning_mode-val is deprecated.  Use multi_agent_planner_node-srv:planning_mode instead.")
  (planning_mode m))

(cl:ensure-generic-function 'allocated_planning_time-val :lambda-list '(m))
(cl:defmethod allocated_planning_time-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:allocated_planning_time-val is deprecated.  Use multi_agent_planner_node-srv:allocated_planning_time instead.")
  (allocated_planning_time m))

(cl:ensure-generic-function 'swarm_start-val :lambda-list '(m))
(cl:defmethod swarm_start-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:swarm_start-val is deprecated.  Use multi_agent_planner_node-srv:swarm_start instead.")
  (swarm_start m))

(cl:ensure-generic-function 'swarm_goal-val :lambda-list '(m))
(cl:defmethod swarm_goal-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:swarm_goal-val is deprecated.  Use multi_agent_planner_node-srv:swarm_goal instead.")
  (swarm_goal m))

(cl:ensure-generic-function 'tolerance-val :lambda-list '(m))
(cl:defmethod tolerance-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:tolerance-val is deprecated.  Use multi_agent_planner_node-srv:tolerance instead.")
  (tolerance m))

(cl:ensure-generic-function 'initial_eps-val :lambda-list '(m))
(cl:defmethod initial_eps-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:initial_eps-val is deprecated.  Use multi_agent_planner_node-srv:initial_eps instead.")
  (initial_eps m))

(cl:ensure-generic-function 'final_eps-val :lambda-list '(m))
(cl:defmethod final_eps-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:final_eps-val is deprecated.  Use multi_agent_planner_node-srv:final_eps instead.")
  (final_eps m))

(cl:ensure-generic-function 'dec_eps-val :lambda-list '(m))
(cl:defmethod dec_eps-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:dec_eps-val is deprecated.  Use multi_agent_planner_node-srv:dec_eps instead.")
  (dec_eps m))

(cl:ensure-generic-function 'sbpl_planner-val :lambda-list '(m))
(cl:defmethod sbpl_planner-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:sbpl_planner-val is deprecated.  Use multi_agent_planner_node-srv:sbpl_planner instead.")
  (sbpl_planner m))

(cl:ensure-generic-function 'planner_type-val :lambda-list '(m))
(cl:defmethod planner_type-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:planner_type-val is deprecated.  Use multi_agent_planner_node-srv:planner_type instead.")
  (planner_type m))

(cl:ensure-generic-function 'meta_search_type-val :lambda-list '(m))
(cl:defmethod meta_search_type-val ((m <GetSwarmPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:meta_search_type-val is deprecated.  Use multi_agent_planner_node-srv:meta_search_type instead.")
  (meta_search_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSwarmPlan-request>) ostream)
  "Serializes a message object of type '<GetSwarmPlan-request>"
  (cl:let* ((signed (cl:slot-value msg 'planning_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'allocated_planning_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swarm_start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'swarm_start))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swarm_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'swarm_goal))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tolerance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'initial_eps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'final_eps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dec_eps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'sbpl_planner)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'planner_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'meta_search_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSwarmPlan-request>) istream)
  "Deserializes a message object of type '<GetSwarmPlan-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planning_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'allocated_planning_time) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swarm_start) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swarm_start)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swarm_goal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swarm_goal)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_eps) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'final_eps) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dec_eps) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sbpl_planner) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planner_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'meta_search_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSwarmPlan-request>)))
  "Returns string type for a service object of type '<GetSwarmPlan-request>"
  "multi_agent_planner_node/GetSwarmPlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSwarmPlan-request)))
  "Returns string type for a service object of type 'GetSwarmPlan-request"
  "multi_agent_planner_node/GetSwarmPlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSwarmPlan-request>)))
  "Returns md5sum for a message object of type '<GetSwarmPlan-request>"
  "a841673a82e79b1e83929033134098f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSwarmPlan-request)))
  "Returns md5sum for a message object of type 'GetSwarmPlan-request"
  "a841673a82e79b1e83929033134098f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSwarmPlan-request>)))
  "Returns full string definition for message of type '<GetSwarmPlan-request>"
  (cl:format cl:nil "int8 planning_mode~%float64 allocated_planning_time~%~%float64[] swarm_start~%float64[] swarm_goal~%~%~%float64 tolerance~%~%float64 initial_eps~%float64 final_eps~%float64 dec_eps~%~%int8 sbpl_planner~%int8 planner_type~%int8 meta_search_type~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSwarmPlan-request)))
  "Returns full string definition for message of type 'GetSwarmPlan-request"
  (cl:format cl:nil "int8 planning_mode~%float64 allocated_planning_time~%~%float64[] swarm_start~%float64[] swarm_goal~%~%~%float64 tolerance~%~%float64 initial_eps~%float64 final_eps~%float64 dec_eps~%~%int8 sbpl_planner~%int8 planner_type~%int8 meta_search_type~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSwarmPlan-request>))
  (cl:+ 0
     1
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swarm_start) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swarm_goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSwarmPlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSwarmPlan-request
    (cl:cons ':planning_mode (planning_mode msg))
    (cl:cons ':allocated_planning_time (allocated_planning_time msg))
    (cl:cons ':swarm_start (swarm_start msg))
    (cl:cons ':swarm_goal (swarm_goal msg))
    (cl:cons ':tolerance (tolerance msg))
    (cl:cons ':initial_eps (initial_eps msg))
    (cl:cons ':final_eps (final_eps msg))
    (cl:cons ':dec_eps (dec_eps msg))
    (cl:cons ':sbpl_planner (sbpl_planner msg))
    (cl:cons ':planner_type (planner_type msg))
    (cl:cons ':meta_search_type (meta_search_type msg))
))
;//! \htmlinclude GetSwarmPlan-response.msg.html

(cl:defclass <GetSwarmPlan-response> (roslisp-msg-protocol:ros-message)
  ((stats_field_names
    :reader stats_field_names
    :initarg :stats_field_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (stats
    :reader stats
    :initarg :stats
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetSwarmPlan-response (<GetSwarmPlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSwarmPlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSwarmPlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_planner_node-srv:<GetSwarmPlan-response> is deprecated: use multi_agent_planner_node-srv:GetSwarmPlan-response instead.")))

(cl:ensure-generic-function 'stats_field_names-val :lambda-list '(m))
(cl:defmethod stats_field_names-val ((m <GetSwarmPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:stats_field_names-val is deprecated.  Use multi_agent_planner_node-srv:stats_field_names instead.")
  (stats_field_names m))

(cl:ensure-generic-function 'stats-val :lambda-list '(m))
(cl:defmethod stats-val ((m <GetSwarmPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_planner_node-srv:stats-val is deprecated.  Use multi_agent_planner_node-srv:stats instead.")
  (stats m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSwarmPlan-response>) ostream)
  "Serializes a message object of type '<GetSwarmPlan-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stats_field_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'stats_field_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'stats))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSwarmPlan-response>) istream)
  "Deserializes a message object of type '<GetSwarmPlan-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stats_field_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stats_field_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stats) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stats)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSwarmPlan-response>)))
  "Returns string type for a service object of type '<GetSwarmPlan-response>"
  "multi_agent_planner_node/GetSwarmPlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSwarmPlan-response)))
  "Returns string type for a service object of type 'GetSwarmPlan-response"
  "multi_agent_planner_node/GetSwarmPlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSwarmPlan-response>)))
  "Returns md5sum for a message object of type '<GetSwarmPlan-response>"
  "a841673a82e79b1e83929033134098f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSwarmPlan-response)))
  "Returns md5sum for a message object of type 'GetSwarmPlan-response"
  "a841673a82e79b1e83929033134098f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSwarmPlan-response>)))
  "Returns full string definition for message of type '<GetSwarmPlan-response>"
  (cl:format cl:nil "~%~%string[] stats_field_names~%float64[] stats~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSwarmPlan-response)))
  "Returns full string definition for message of type 'GetSwarmPlan-response"
  (cl:format cl:nil "~%~%string[] stats_field_names~%float64[] stats~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSwarmPlan-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stats_field_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stats) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSwarmPlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSwarmPlan-response
    (cl:cons ':stats_field_names (stats_field_names msg))
    (cl:cons ':stats (stats msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSwarmPlan)))
  'GetSwarmPlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSwarmPlan)))
  'GetSwarmPlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSwarmPlan)))
  "Returns string type for a service object of type '<GetSwarmPlan>"
  "multi_agent_planner_node/GetSwarmPlan")