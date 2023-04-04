(load "simulator")

(progn(shadowing-import 'sim:name)
      (shadowing-import 'sim:simulator)
      (shadowing-import 'sim:object)
      (shadowing-import 'sim:size)
      (shadowing-import 'sim:location))

(use-package 'sim)


;;; NODE class for defining various nodes in the maze
(defclass node()
  ((parent :initarg :parent :initform nil :accessor parent)
   (row-col :initarg :row-col :initform nil :accessor row-col)
   (g :initarg :g :initform 0 :accessor g)
   (h :initarg :h :initform 0 :accessor h)
   (f :initarg :f :initform 0 :accessor f)))

(defun make-node (&key parent row-col)
  (make-instance 'node :parent parent :row-col row-col))


;;; simple reflex agent
(defclass reflex-agent (robot) ())

(defmethod agent-program ((self reflex-agent) percept)
  ;; move forward or right indefinitely
  (if (cadr (assoc :front-sensor percept) )
      :right :forward))

(defclass model-based-agent (robot) 
  ((back-flag :initarg :back-flag :initform 0 :accessor back-flag)))

(defmethod agent-program ((self model-based-agent) percept)
  ;; keeps going forward and right until its in the corner
  ;; goes back and right to go around the false corners
  ;; if tries going around for 5 times, 
  ;; it is a real edge, move forward and hit the corner
  (let ((back-flag (back-flag self)))
	(if (< back-flag 6)   ;check up until 5 false corners
	    (progn
	      ;; if nothing ahead, keep moving forward
	      (if (and (eql (cadr(assoc :forward-sensor percept)) nil) (eql (cadr(assoc :right-bump percept)) nil) (equal back-flag 0))
		  (return-from agent-program :forward))
	      ;; if something ahead, move right
	      (if (and (eql (cadr(assoc :forward-sensor percept)) t) (eql (cadr(assoc :right-bump percept)) nil))
		    (return-from agent-program :right))
	      ;; if corner detected, try going around it
	      (if (and (eql (cadr(assoc :forward-sensor percept)) t) (eql (cadr(assoc :right-bump percept)) t))
		  (progn
		    (incf (back-flag self))
		    (return-from agent-program :backward)))
	      ;; after going back, go right, to go around the obstacle
	      (if (and (eql (cadr(assoc :forward-sensor percept)) nil) (eql (cadr(assoc :right-bump percept)) nil) (> back-flag 0))
		  (progn
		    (decf (back-flag self))
		    (return-from agent-program :right)))
	      ;; if cannot go right, go back again to check if it false edge
	      (if (and (eql (cadr(assoc :forward-sensor percept)) nil) (eql (cadr(assoc :right-bump percept)) t))
		  (progn
		    (setf (back-flag self) (+ (back-flag self) 2))
		    (return-from agent-program :backward))))
	    ;; keep going forward since real edge is found
	    (return-from agent-program :forward))))


;;;> hill-climbing agent with manhattan distance to the goal nose as heuristic
(defclass hill-climbing-agent (robot) 
  ((start :initarg :start :initform '(1 1) :accessor start)
   (goal :initarg :goal :initform '(25 25) :accessor goal)))

(defmethod agent-program ((self hill-climbing-agent) percept)
  (let (start-node end-node current-node (robot-loc (location self)) children best-loc)
    (setq start-node (make-instance 'node :row-col robot-loc))
    (setq end-node (make-instance 'node :row-col (goal self)))
    (setq current-node start-node)
    (setf (f current-node) (manhattan-heuristic end-node current-node))
    ;; generate children positions, from left, right, top and bottom
    (setq children (generate-children current-node children)) ; generate-children function

    ;; if any percepts t from last action, means last best child should be removed
    ;; i.e. there is obstacle in the position of last best child
    (if (or (cadr (assoc :front-bump percept)) (cadr (assoc :rear-bump percept)) (cadr(assoc :right-bump percept)) (cadr(assoc :left-bump percept)))
	(progn
	  ;; checking if it reached the goal
	  (if (equal best-loc (row-col end-node))
	      (return-from agent-program :nop))
	  (setq children (remove (best-child children current-node end-node) children))))

    ;; choose child closest to the end node using manhattan distance
    (setq current-node (best-child children current-node end-node))

    (setq best-loc (row-col current-node)) ; best position to move to
    
    ;; setting the robot  orientation to NORTH before doing other actions
    (if (equal (orientation self) :west)
	(return-from agent-program :turn-right))
    (if (equal (orientation self) :east)
	(return-from agent-program :turn-left))
    (if (equal (orientation self) :south)
	(return-from agent-program :turn-right))

    (progn (let (row-sub col-sub)
	     ;; subtraction coordinates of robot location and next location to 
	     ;; check whick direction to move in
	     (setq row-sub (- (elt best-loc 0) (elt robot-loc 0)))
	     (setq col-sub (- (elt best-loc 1) (elt robot-loc 1)))
	     (if (and (equal row-sub 0) (equal col-sub -1))
		 (return-from agent-program :backward))
	     (if (and (equal row-sub 0) (equal col-sub 1))
		 (return-from agent-program :forward))
	     (if (and (equal row-sub -1) (equal col-sub 0))
		 (return-from agent-program :left))
	     (if (and (equal row-sub 1) (equal col-sub 0))
		 (return-from agent-program :right))
	     (if (and (equal row-sub 0) (equal col-sub 0))
		 (return-from agent-program :nop))))))


;;;> uniform cost search agent with no heiristic function, h=0 i.e. f=g
(defclass uniform-cost-search-agent (robot)
  ((start :initarg :start :initform '(1 1) :accessor start)
   (goal :initarg :goal :initform '(25 25) :accessor goal)
   (maze :initarg :maze :initform (make-array '(25 25) :initial-element 0) :accessor maze)
   (path :initarg :path :initform nil :accessor path)
   (nodes-created :initarg :nodes-created :initform 0 :accessor nodes-created)
   ))

;; creating 2D array to represent the map of the simulator world
(defun create-map(maze robot-loc)
  (let (obstacles-list)
    (setq obstacles-list (object-locations sim))
    (setq obstacles-list (remove robot-loc obstacles-list))
    (dolist (obs-loc obstacles-list)
      ;;set 1 for obstacle
      (setf (aref maze (- (elt obs-loc 0) 1) (- (elt obs-loc 1) 1)) 1)) 
    (return-from create-map maze)))

(defmethod agent-program ((self uniform-cost-search-agent) percept)  
  (let ((path (path self))  (maze (maze self)) start-node end-node (robot-loc (location self)) (goal-loc (goal self)) next-loc nodes)
    ;; doing -1 to the robot's location for representation in 2D array 
    ;; whose indesx starts from 0
    (setq robot-loc (list (- (elt robot-loc 0) 1) (- (elt robot-loc 1) 1)))
    (setq goal-loc (list (- (elt goal-loc 0) 1) (- (elt goal-loc 1) 1)))

    ;; do this only the first time i.e. call the astar search function
    (if (and (null path) (not (equal robot-loc goal-loc)))
	(progn
	  (setf (maze self) (create-map maze robot-loc))
	  ;; creating node instances for start and end node
	  (setq start-node (make-instance 'node :row-col robot-loc))
	  (setq end-node (make-instance 'node :row-col goal-loc)) ; goal position
	  ;; actual search algorithm
	  (setf (values path nodes) (ucs (maze self) start-node end-node))
	  (setf (nodes-created self) nodes)
	  (setq path (remove (car path) path)) ; remove first position 
	  (setf (path self) path)
	  )) 

    ;;set orientation to north before doing other actions
    (if (not (equal (orientation self) :north))
	(return-from agent-program (turn-north (orientation self))))
    
    ;; if already in the goal position, no search
    (if (null path)
	(return-from agent-program :nop)
	(setq next-loc (car path)))
    (setq path (remove next-loc path))
    (setf (path self) path)

    ;; perform the next action
    (move next-loc robot-loc)
   
    ))

;;;> astar search agent with manhattan heuristic
(defclass astar-search-agent-h1 (robot)
  ((start :initarg :start :initform '(1 1) :accessor start)
   (goal :initarg :goal :initform '(25 25) :accessor goal)
   ;; creating 2D array to represent the map of the simulator world
   (maze :initarg :maze :initform (make-array '(25 25) :initial-element 0) :accessor maze)
   (path :initarg :path :initform nil :accessor path)
   (nodes-created :initarg :nodes-created :initform 0 :accessor nodes-created)
   ))

(defmethod agent-program ((self astar-search-agent-h1) percept)
  (let ((path (path self))  (maze (maze self)) start-node end-node (robot-loc (location self)) (goal-loc (goal self)) next-loc nodes)
    ;; doing -1 to the robot's location for representation in 2D array 
    ;; whose indesx starts from 0
    (setq robot-loc (list (- (elt robot-loc 0) 1) (- (elt robot-loc 1) 1)))
    (setq goal-loc (list (- (elt goal-loc 0) 1) (- (elt goal-loc 1) 1)))

    ;; do this only the first time i.e. call the astar search function
    (if (and (null path) (not (equal robot-loc goal-loc)))
	(progn
	  (setf (maze self) (create-map maze robot-loc))
	  ;; creating node instances for start and end node
	  (setq start-node (make-instance 'node :row-col robot-loc))
	  (setq end-node (make-instance 'node :row-col goal-loc)) ; goal position
	  ;; actual search algorithm
	  (setf (values path nodes) (astar (maze self) start-node end-node 1))
	  (setf (nodes-created self) nodes)
	  (setq path (remove (car path) path)) ; remove first position 
	  (setf (path self) path)
	  )) 

    ;;set orientation to north before doing other actions
    (if (not (equal (orientation self) :north))
	(return-from agent-program (turn-north (orientation self))))
    
    ;; if already in the goal position, no search
    (if (null path)
	(return-from agent-program :nop)
	(setq next-loc (car path)))
    (setq path (remove next-loc path))
    (setf (path self) path)

    ;; perform the next action
    (move next-loc robot-loc)

    ))

;;;> astar search agent with euclidean distance  heuristic
(defclass astar-search-agent-h2 (robot)
  ((start :initarg :start :initform '(1 1) :accessor start)
   (goal :initarg :goal :initform '(25 25) :accessor goal)
   ;; creating 2D array to represent the map of the simulator world
   (maze :initarg :maze :initform (make-array '(25 25) :initial-element 0) :accessor maze)
   (path :initarg :path :initform nil :accessor path)
   (nodes-created :initarg :nodes-created :initform 0 :accessor nodes-created)
   ))
	

(defmethod agent-program ((self astar-search-agent-h2) percept)
 (let ((path (path self))  (maze (maze self)) start-node end-node (robot-loc (location self)) (goal-loc (goal self)) next-loc nodes)
    ;; doing -1 to the robot's location for representation in 2D array 
    ;; whose indesx starts from 0
    (setq robot-loc (list (- (elt robot-loc 0) 1) (- (elt robot-loc 1) 1)))
    (setq goal-loc (list (- (elt goal-loc 0) 1) (- (elt goal-loc 1) 1)))

    ;; do this only the first time i.e. call the astar search function
    (if (and (null path) (not (equal robot-loc goal-loc)))
	(progn
	  (setf (maze self) (create-map maze robot-loc))
	  ;; creating node instances for start and end node
	  (setq start-node (make-instance 'node :row-col robot-loc))
	  (setq end-node (make-instance 'node :row-col goal-loc)) ; goal position
	  ;; actual search algorithm
	  (setf (values path nodes) (astar (maze self) start-node end-node 0))
	  (setf (nodes-created self) nodes)
	  (setq path (remove (car path) path)) ; remove first position 
	  (setf (path self) path)
	  )) 

    ;;set orientation to north before doing other actions
    (if (not (equal (orientation self) :north))
	(return-from agent-program (turn-north (orientation self))))
    
    ;; if already in the goal position, no search
    (if (null path)
	(return-from agent-program :nop)
	(setq next-loc (car path)))
    (setq path (remove next-loc path))
    (setf (path self) path)

    ;; perform the next action
    (move next-loc robot-loc)

    ))


;;; defining function for uniform cost search algorithm
(defun ucs(maze start-node end-node)
  ;; create opened and closed lists
  (let ((opened-list (list nil)) (closed-list (list nil)) (current-node nil) (children (list nil)))
    (setq opened-list (append opened-list(list start-node))) ; add start-node in the opened-list
    (setq opened-list (remove nil opened-list))
    ;; loop until opened-node list is emplty
    (loop while (not (null opened-list))
       do (progn
	   ;; get current node from the opened-list
	   (setq current-node (car opened-list))
	   (setq children nil)
	   ;; check for least heuristic value f
	   (setq current-node (least-heuristic opened-list current-node))
	  
	   ;; expand current node, remove from the opened-list, and add to closed-node
	   (setq opened-list(remove current-node opened-list))
 	   (setq closed-list (append closed-list (list current-node)))
	   (setq closed-list (remove nil closed-list))
	   ;; stop if goal is found and return complete path
	   (if (equal (row-col current-node) (row-col end-node))
	       (return-from ucs (values (find-path current-node) (length closed-list))))

	   ;; explore possible children nodes in row-cols left, right, bottom, and top 
	   ;; and generate those children
	   (setq children (generate-children-maze maze current-node children))
	   (setq children (remove nil children))
	   
	   ;; loop through the children nodes
	   (dolist (child children)
	     ;; check child is not in the closed list
	     (if (not (child-in-list? closed-list child))
		 (progn
		   ;; calculate total heuristic value
		   (setf (g child) (+ (g current-node) 1))
		   (setf (f child) (g child))
		   ;; add child to opened-list, if already present, check heuristic and 
		   ;; keep the child with least heuristic function value
		   (setq opened-list (add-child-opened-list opened-list child)) )))
	  ))))


;;; defining function for astar search algorithm
(defun astar (maze start-node end-node heuristic)
  ;; create opened and closed lists
  (let ((opened-list (list nil)) (closed-list (list nil)) (current-node nil) (children (list nil)))
    (setq opened-list (append opened-list(list start-node))) ; add start-node in the opened-list
    (setq opened-list (remove nil opened-list))
    ;; loop until opened-node list is emplty
    (loop while (not (null opened-list))
       do (progn
	   ;; get current node from the opened-list
	   (setq current-node (car opened-list))
	   (setq children nil)
	   ;; check for least heuristic value f
	   (setq current-node (least-heuristic opened-list current-node))
	  
	   ;; expand current node, remove from the opened-list, and add to closed-node
	   (setq opened-list(remove current-node opened-list))
 	   (setq closed-list (append closed-list (list current-node)))
	   (setq closed-list (remove nil closed-list))
	   ;; stop if goal is found and return complete path
	   (if (equal (row-col current-node) (row-col end-node))
	       (return-from astar (values (find-path current-node) (length closed-list))))

	   ;; explore possible children nodes in row-cols left, right, bottom, and top   
	   ;; and generate those children
	   (setq children (generate-children-maze maze current-node children))
	   (setq children (remove nil children))
	   
	   ;; loop through the children nodes
	   (dolist (child children)
	     ;; check child is not in the closed list
	     (if (not (child-in-list? closed-list child))
		 (progn
		   (setf (g child) (+ (g current-node) 1))
		   ;; calculating manhattan heuristic function
		   (if (equal heuristic 1)
		       (setf (h child) (manhattan-heuristic end-node child))
		       (setf (h child) (euclidean-heuristic end-node child)))
		   ;; calculate total heuristic value
		   (setf (f child) (+ (g child) (h child)))
		   ;; add child to opened-list, if already present, check heuristic and 
		   ;; keep the child with least heuristic function value
		   (setq opened-list (add-child-opened-list opened-list child)) )))
	  ))))


;;; function to set the robot  orientation to NORTH before doing other actions
(defun turn-north(orientation)
  (if (equal orientation :west)
	(return-from turn-north :turn-right))
  (if (equal orientation :east)
	(return-from turn-north :turn-left))
  (if (equal orientation :south)
	(return-from turn-north :turn-right)))

;;; function to return the moves in the agent program
(defun move (next-loc robot-loc)
  ;; subtract coordinates of robot location and next location to 
  ;; check whick direction to move in
  (progn (let (row-sub col-sub)
	     (setq row-sub (- (elt next-loc 0) (elt robot-loc 0)))
	     (setq col-sub (- (elt next-loc 1) (elt robot-loc 1)))
	     (if (and (equal row-sub 0) (equal col-sub -1))
		 (return-from move :backward))
	     (if (and (equal row-sub 0) (equal col-sub 1))
		 (return-from move :forward))
	     (if (and (equal row-sub -1) (equal col-sub 0))
		 (return-from move :left))
	     (if (and (equal row-sub 1) (equal col-sub 0))
		 (return-from move :right))
	     (if (and (equal row-sub 0) (equal col-sub 0))
		 (return-from move :nop)))))

;;; function for checking the least heuristic value f in the opened list
(defun least-heuristic (opened-list current-node)
  (dolist (ele opened-list)
    (if (< (f ele) (f current-node)) (setq current-node ele)))
  current-node)
    

;;; function for finding the best child(least manhattan distance) for hill climbing agent
(defun best-child (children current-node end-node)
  (dolist (child children)
      (setf (f child) (manhattan-heuristic end-node child)) ; chosen heuristic: manhattan
      (if (< (f child) (f current-node))
	  (setq current-node child)))
  current-node)

;;;function to find the final path
(defun find-path (current-node)
  (let ((path (list nil)) (current current-node)) ; create a variable to store path
    (loop while (not (null current))
       do (progn (setq path (append path (list (row-col current))))
	    ;; store current node and all its successive parent nodes for
	    ;; the complete path
		 (setq current (parent current))))
    (setq path (reverse path)) ; reverse the path so that it's start-node to end-node
    (setq path (remove nil path))
    ;; return the complete path for astar search
    (return-from find-path path)))

;;; function to generate children i.e. direct neighbors 
(defun generate-children (current-node children)
  (let ((new-row-cols (list '(0 -1) '(0 1) '(1 0) '(-1 0))) node-row-col new-node)
    (dolist (new-row-col new-row-cols)
      ;; get a new node row-col out of the possible row-cols top, bottom, left and right
      (setq node-row-col (list (+ (elt (row-col current-node) 0) (elt new-row-col 0)) (+ (elt (row-col current-node) 1) (elt new-row-col 1))))
      ;; computing the max row index and max column index for the maze
      (setq new-node (make-instance 'node :parent current-node :row-col node-row-col))
      ;; create the new node to be expanded 
      (setq children (append children (list new-node)))
      (setq children (remove nil children)))
    (return-from generate-children children)))

;;; function to generate children in the maze i.e the map of the robot world
(defun generate-children-maze (maze current-node children)
  (let ((new-row-cols (list '(0 -1) '(0 1) '(1 0) '(-1 0))) node-row-col node-row node-col max-row max-col new-node)
    (dolist (new-row-col new-row-cols)
      ;; get a new node row-col out of the possible row-cols top, bottom, left and right
      (setq node-row-col (list (+ (elt (row-col current-node) 0) (elt new-row-col 0)) (+ (elt (row-col current-node) 1) (elt new-row-col 1))))
      (setq node-row (elt node-row-col 0))
      (setq node-col (elt node-row-col 1)) ; get the row and column of the new node-row-col
      ;; computing the max row index and max column index for the maze
      (setq max-row (- (array-dimension maze 0) 1))
      (setq max-col (- (array-dimension maze 1) 1))
      ;; check if the new row-col is within the walls of the maze
      ;; check if row within the range
      (if (and (not(> node-row max-row)) (> node-row -1))
	  (progn
	    ;; check if column within the range
	    (if (and (not (> node-col max-col)) (> node-col -1))
		(progn
		  ;; check there's no obstacle, 1 is obstacle, 0 is no obstacle
		  (if (equal (aref maze node-row node-col) 0)
		      (progn
			(setq new-node (make-instance 'node :parent current-node :row-col node-row-col))
			;; create the new node to be expanded 
			(setq children (append children (list new-node)))
			(setq children (remove nil children)) )))))) ))
    (return-from generate-children-maze children))

;;; function to check of child is in the search-list
(defun child-in-list? (search-list child)
  (if (not (null search-list))
      (dolist (ele search-list)
	(if (equal (row-col child) (row-col ele))
	    (return-from child-in-list? T)))) ; returns T if child id found
      (return-from child-in-list? nil)) ; nil if child wasn't found

;;; function to calculate manhattan distance
(defun manhattan-heuristic (end-node child)
  (let ((end-x-cor (elt (row-col end-node) 0))
	(end-y-cor (elt (row-col end-node) 1))
	(child-x-cor (elt (row-col child) 0))
	(child-y-cor (elt (row-col child) 1))
	(manhattan-dist 0))
    (setq manhattan-dist (+ (abs (- end-x-cor child-x-cor)) (abs(- end-y-cor child-y-cor))))
    (return-from manhattan-heuristic manhattan-dist)))

;;; function to calculate euclidean distance
(defun euclidean-heuristic (end-node child)
  (let ((end-x-cor (elt (row-col end-node) 0))
	(end-y-cor (elt (row-col end-node) 1))
	(child-x-cor (elt (row-col child) 0))
	(child-y-cor (elt (row-col child) 1))
	(euclidean-dist 0))
    (setq euclidean-dist (sqrt (+ (expt (- end-x-cor child-x-cor) 2) (expt (- end-y-cor child-y-cor) 2))))
    (return-from euclidean-heuristic euclidean-dist)))

;;; function to add the possible children to the opened-list
(defun add-child-opened-list (opened-list child)
  (if (or (null opened-list) (equal opened-list (list nil)))
      (progn
	(setq opened-list (append opened-list (list child))) 
	(return-from add-child-opened-list (remove nil opened-list))))
  ;; check if node already present in opened list with greater
  ;; heuristic, replace
  (dolist (opened-node opened-list)
    (if (equal (row-col child) (row-col opened-node))
	(if (> (g opened-node) (g child))
	    (progn
	      (setq opened-list (remove opened-node opened-list))
	      (setq opened-list (append opened-list (list child)))
	      (return-from add-child-opened-list (remove nil opened-list)))
	    (return-from add-child-opened-list opened-list))))
  (setq opened-list (append opened-list (list child)))
  (return-from add-child-opened-list (remove nil opened-list)))