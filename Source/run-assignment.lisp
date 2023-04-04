(load "simulator")

(progn(shadowing-import 'sim:name)
      (shadowing-import 'sim:simulator)
      (shadowing-import 'sim:object)
      (shadowing-import 'sim:size)
      (shadowing-import 'sim:location)
      (shadowing-import 'message:msg)
      (shadowing-import 'message:destination))

(use-package 'sim)
(use-package 'message)
(use-package 'newsymbol)

(load "search-assignment")
(load "other-domain")

(print "LOADING PACKAGES AND FILES COMPLETE")

(defun run-robots()
  (let ((nodes-created-list (list nil)))
    (print "SIMPLE-REFLEX AGENT")
    (run-reflex-robot)

    (print "MODEL-BASED AGENT")
    (run-model-based-robot)



    ;; HILL CLIMBING AGENT
    (print "HILL CLIMBING AGENT")
    (print "SIMULATED IN 21 RANDOM ENVIRONMENTS")
    (print "Start position always: (1 1)") ;made consistent for analysing nodes created

    (print "FOR 0 OBSTACLES  AND GOAL POSITION: (25 25")
    (simulate :agent-class 'hill-climbing-agent :sketch? t :obstacles '(0))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (25 25")
    (simulate :agent-class 'hill-climbing-agent :sketch? t :obstacles '(50))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (13 13")
    (simulate :agent-class 'hill-climbing-agent :sketch? t :obstacles '(10 20 30 40 50) :goal-position '(13 13))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (25 10")
    (simulate :agent-class 'hill-climbing-agent :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(25 10) :fake-corners '((1 1) (25 1) (25 25)))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (17 23")
    (simulate :agent-class 'hill-climbing-agent :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(17 23) :fake-corners '((1 1) (25 1) (25 25)))



    ;; UNIFORM COST AGENT
    (print "UNIFORM COST SEARCH AGENT")
    (print "SIMULATED IN 21 RANDOM ENVIRONMENTS")
    (print "Start position always: (1 1)") ;made consistent for analysing nodes created

    (print "FOR 0 OBSTACLES  AND GOAL POSITION: (25 25")
    (simulate :agent-class 'uniform-cost-search-agent :sketch? t :obstacles '(0))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (25 25")
    (simulate :agent-class 'uniform-cost-search-agent :sketch? t :obstacles '(50))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (13 13")
    (simulate :agent-class 'uniform-cost-search-agent :sketch? t :obstacles '(10 20 30 40 50) :goal-position '(13 13))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (25 10")
    (simulate :agent-class 'uniform-cost-search-agent :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(25 10) :fake-corners '((1 1) (25 1) (25 25)))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (17 23")
    (simulate :agent-class 'uniform-cost-search-agent :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(17 23) :fake-corners '((1 1) (25 1) (25 25)))




    ;; ASTAR MANHATTAN
    (print "ASTAR SEARCH AGENT WITH MANHATTAN HEURISTIC FUNCTION")
    (print "SIMULATED IN 21 RANDOM ENVIRONMENTS")
    (print "Start position always: (1 1)") ;made consistent for analysing nodes created

    (print "FOR 0 OBSTACLES  AND GOAL POSITION: (25 25")
    (simulate :agent-class 'astar-search-agent-h1 :sketch? t :obstacles '(0))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (25 25")
    (simulate :agent-class 'astar-search-agent-h1 :sketch? t :obstacles '(50))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (13 13")
    (simulate :agent-class 'astar-search-agent-h1 :sketch? t :obstacles '(10 20 30 40 50) :goal-position '(13 13))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (25 10")
    (simulate :agent-class 'astar-search-agent-h1 :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(25 10) :fake-corners '((1 1) (25 1) (25 25)))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (17 23")
    (simulate :agent-class 'astar-search-agent-h1 :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(17 23) :fake-corners '((1 1) (25 1) (25 25)))




    ;; ASTAR EUCLIDEAN
    (print "ASTAR SEARCH AGENT WITH EUCLIDEAN HEURISTIC FUNCTION")
    (print "SIMULATED IN 21 RANDOM ENVIRONMENTS")
    (print "Start position always: (1 1)") ;made consistent for analysing nodes created

    (print "FOR 0 OBSTACLES  AND GOAL POSITION: (25 25")
    (simulate :agent-class 'astar-search-agent-h2 :sketch? t :obstacles '(0))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (25 25")
    (simulate :agent-class 'astar-search-agent-h2 :sketch? t :obstacles '(50))

    (print "FOR RANDOM OBSTACLES, GOAL POSITION: (13 13")
    (simulate :agent-class 'astar-search-agent-h2 :sketch? t :obstacles '(10 20 30 40 50) :goal-position '(13 13))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (25 10")
    (simulate :agent-class 'astar-search-agent-h2 :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(25 10) :fake-corners '((1 1) (25 1) (25 25)))

    (print "FOR RANDOM OBSTACLES AND FAKE CORNERS, GOAL POSITION: (17 23")
    (simulate :agent-class 'astar-search-agent-h2 :sketch? t :obstacles '(20 30 40 50 60) :goal-position '(17 23) :fake-corners '((1 1) (25 1) (25 25)))

    ))


;;; function to run the uniform cost search and astar on the travelling salesman graph
(defun run-other ()
  (let* (graph start-node end-node path )
    (setq graph (create-graph));graph of 3 cities
    (setq start-node (make-instance 'node :name 0))
    (setq end-node (make-instance 'node :name (- (array-dimension graph 0) 1)))

    ;; using UCS for travelling salesman
    (print "UCS generated path for travelling salesman")
    (setq path (ucs-other graph start-node end-node))
    (setq path (mapcar #'conversion path))
    (print path)
    
    ;;using Astar algorithm for travelling salesman
    (print "Astar generated path for travelling salesman")
    (setq path (astar-other graph start-node end-node))
    (setq path (mapcar #'conversion path))
    (print path)
))


;; function to create simulator, add robot and run 
(defun simulate(&key agent-class sketch? obstacles (goal-position '(25 25))  fake-corners)
  (let (sim sim-robo) ;(nodes-created-list (list nil)))
  (dolist (ele obstacles)
      (progn 
	(setq sim (create-simulator :size '(25 25) :num-obstacles ele :obstacle-locations fake-corners))
	(setq sim-robo (make-instance agent-class))
	(setf (goal sim-robo) goal-position)
	(format nil "Running ~a in simulator with ~a random obstacles" agent-class ele)
	(format nil "Goal Position: ~a" (goal sim-robo))
	;; checkif there's an obstacle in the start postiion (1 1)
	(if (obstacle-at-loc? sim '(1 1)) ; if true, use random start point
	    (add-robot sim :robot sim-robo :random-location t :random-orientation t)
	    (add-robot sim :robot sim-robo :location '(1 1) :random-location nil :random-orientation t))
	(run sim :for 50  :sketch-each sketch?)
	;(format nil "Nodes created for ~a obstacles: ~a" ele (nodes-created sim-robo))
	;(setq nodes-created-list (append nodes-created-list (list (nodes-created sim-robo))))
	))))
  ;(return-from simulate (remove nil nodes-created-list))))

;;; function to check if there is an obstacle at the given location in simulator
(defun obstacle-at-loc?(sim loc)
  (dolist (ele (object-locations sim))
    (if (equal ele loc)
	(return-from obstacle-at-loc? T)))
  (return-from obstacle-at-loc? nil))


(defun run-model-based-robot()
  (let (sim-1 sim-2 sim-3 model-based-robot)
    ;; NO OBSTACLES
    (print "Running Model based Agent in simulator with no obstacles")
    (setq model-based-robot (make-instance 'model-based-agent))
    (setq sim-1 (create-simulator :size '(25 25)))
    (print "With random location and orientation")
    (progn (clear sim-1)
    	   (add-robot sim-1 :robot model-based-robot  :random-location t :random-orientation t)
    	   (run sim-1 :for 100  :sketch-each t))
   
    (print "With initial location (1 1) and random orientation")
    (progn (clear sim-1)
    	   (add-robot sim-1 :robot model-based-robot  :random-location nil :location '(1 1) :random-orientation t)
    	   (run sim-1 :for 100 :sketch-each t))

    (print "With random location and initial orientation :WEST")
    (progn (clear sim-1)
    	   (add-robot sim-1 :robot model-based-robot  :random-location t :orientation :west :random-orientation nil)
    	   (run sim-1 :for 100 :sketch-each t))
   
    ;; RANDOM OBSTACLES
    (print "Running Model based Agent in simulator with 10, 15, 20 random obstacles")
    (dolist (ele '(10 15 20))
      (progn
	(setq model-based-robot (make-instance 'model-based-agent))
	(setq sim-2 (create-simulator :size '(25 25) :num-obstacles ele))
	(add-robot sim-2 :robot model-based-robot  :random-location t :random-orientation t)
	(run sim-2 :for 100  :sketch-each t)))
  
    ;; RANDOM OBSTACLES AND FALSE CORNERS
    (print "Running Model based Agent in simulator with 10, 15, 20 random obstacles")
    (dolist (ele '(10 15 20))
      (progn
	(setq model-based-robot (make-instance 'model-based-agent))
    	(setq sim-3 (create-simulator :size '(25 25) :num-obstacles ele :obstacle-locations '((1 1) (25 1) (25 25))))
    	(add-robot sim-3 :robot model-based-robot  :random-location t :random-orientation t)
    	(run sim-3 :for 100  :sketch-each t)))
    ))


(defun run-reflex-robot()
  (let (sim-1 sim-2 sim-3 reflex-robot)
    ;;NO OBSTACLES
    (print "Running Reflex Agent in simulator with no obstacles")
    (setq reflex-robot (make-instance 'reflex-agent))
    (setq sim-1 (create-simulator :size '(25 25)))
    (print "With random location and orientation")
    (progn (clear sim-1)
	   (add-robot sim-1 :robot reflex-robot  :random-location t :random-orientation t)
	   (run sim-1 :for 70  :sketch-each t))
   
    (print "With initial location (1 1) and random orientation")
    (progn (clear sim-1)
	   (add-robot sim-1 :robot reflex-robot  :random-location nil :location '(1 1) :random-orientation t)
	   (run sim-1 :for 70 :sketch-each t))

    (print "With random location and initial orientation :WEST")
    (progn (clear sim-1)
	   (add-robot sim-1 :robot reflex-robot  :random-location t :orientation :west :random-orientation nil)
	   (run sim-1 :for 70 :sketch-each t))
   
    ;; RANDOM OBSTACLES
    (print "Running Reflex Agent in simulator with 10, 15 and 20 random obstacles")
    (dolist (ele '(10 15 20))
      (progn
	(setq sim-2 (create-simulator :size '(25 25) :num-obstacles ele))
	(add-robot sim-2 :robot reflex-robot  :random-location t :random-orientation t)
	(run sim-2 :for 70  :sketch-each t)))
   
    ;; RANDOM OBSTACLES AND FALSE CORNERS
    (print "Running Reflex Agent in simulator with false corners and 10 random obstacles")
    (dolist (ele '(10 15 20))
      (progn
	(setq sim-3 (create-simulator :size '(25 25) :num-obstacles ele :obstacle-locations '((1 1) (25 1) (25 25))))
	(add-robot sim-3 :robot reflex-robot  :random-location t :random-orientation t)
	(run sim-3 :for 70  :sketch-each t)))
    ))