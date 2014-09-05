
(cl:in-package :asdf)

(defsystem "multi_agent_planner_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetSwarmPlan" :depends-on ("_package_GetSwarmPlan"))
    (:file "_package_GetSwarmPlan" :depends-on ("_package"))
  ))