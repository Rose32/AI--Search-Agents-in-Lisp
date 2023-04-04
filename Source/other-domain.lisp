
						    
(defclass node()
  ((name :initarg :name :initform 0 :accessor name)
   (parent :initarg :parent :initform nil :accessor parent)
   (g :initarg :g :initform 0 :accessor g)
   (h :initarg :h :initform 0 :accessor h)
   (f :initarg :f :initform 0 :accessor f)))

;;; create a small graph of three cities using adjacency matrix
;;; since we need a path fthat visits every node once and come back to the initial point
;;; a directed graph is made as explained in the write-up
(defun create-graph ()
  (return-from create-graph (make-array '(17 17) :initial-contents '(( 0 400 500 300 0 0 0 0 0 0 0 0 0 0 0 0 0 )
( 0 0 0 0 300 500 0 0 0 0 0 0 0 0 0 0 0 )
( 0 0 0 0 0 0 300 400 0 0 0 0 0 0 0 0 0 )
( 0 0 0 0 0 0 0 0 500 400 0 0 0 0 0 0 0 )
( 0 0 0 0 0 0 0 0 0 0 400 0 0 0 0 0 0 )
( 0 0 0 0 0 0 0 0 0 0 0 400 0 0 0 0 0 )
( 0 0 0 0 0 0 0 0 0 0 0 0 500 0 0 0 0 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 500 0 0 0 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300 0 0 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300 0 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 500 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 300 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 400 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 500 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 400 )
( 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 )))))

;; (0 400 500 0 0 0 0 0)
;; (0 0 0 300 0 0 0 0)
;; (0 0 0 0 300 0 0 0)
;; (0 0 0 0 0 300 0 0)
;; (0 0 0 0 0 0 300 0)
;; (0 0 0 0 0 0 0 400)
;; (0 0 0 0 0 0 0 500)
;; (0 0 0 0 0 0 0 0)))))
    
;;; function to expand the curretn node and store children nodes
(defun expand-node(graph current-node children)
  (let (new-node)
    (dolist (ele (list 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16))
    (if (> (aref graph (name current-node) ele) 0)
	(progn
	  (setq new-node (make-instance 'node :name ele :parent current-node :g (aref graph (name current-node) ele)))
	  (setq children (append children (list new-node))))))
    (return-from expand-node (remove nil children))))

;;; convert node  numbers to city names
(defun conversion (ele)
  (if (or (equal ele 3) (equal ele 5) (equal ele 7) (equal ele 10) (equal ele 12))
	  (return-from conversion 'd))
  (if (or (equal ele 1) (equal ele 6) (equal ele 8) (equal ele 13) (equal ele 15))
	  (return-from conversion 'b))
  (if (or (equal ele 2) (equal ele 4) (equal ele 9) (equal ele 11) (equal ele 14))
	  (return-from conversion 'c))
  (if (or (equal ele 0)(equal ele 16))
	  (return-from conversion 'a)))

;;; main function to run the uniform cost search and astar on the travelling salesman graph
;; (defun run-other ()
;;   (let* (graph start-node end-node path )
;;     (setq graph (create-graph));graph of 3 cities
;;     (setq start-node (make-instance 'node :name 0))
;;     (setq end-node (make-instance 'node :name (- (array-dimension graph 0) 1)))

;;     ;; using UCS for travelling salesman
;;     (print "UCS generated path for travelling salesman")
;;     (setq path (ucs-other graph start-node end-node))
;;     (setq path (mapcar #'conversion path))
;;     (print path)
    
;;     ;;using Astar algorithm for travelling salesman
;;     (print "Astar generated path for travelling salesman")
;;     (setq path (astar-other graph start-node end-node))
;;     (setq path (mapcar #'conversion path))
;;     (print path)
;; ))

;;; defining function for uniform cost search algorithm
(defun ucs-other(graph start-node end-node)
  ;; create opened and closed lists
  (let ((opened-list (list nil)) (closed-list (list nil)) (current-node nil) (children (list nil)))
    (setq opened-list (append opened-list(list start-node))) ; add start-node in the opened-list
    (setq opened-list (remove nil opened-list))
    ;; loop until opened-node list is emplty
    (loop while (not (null opened-list))
       do (progn
	   ;; get current node from the opened-list
	   (setq current-node (car opened-list))
	   ;; check for least heuristic value f
	   (setq current-node (least-heuristic opened-list current-node))
	  
	   ;; expand current node, remove from the opened-list, and add to closed-node
	   (setq opened-list(remove current-node opened-list))
 	   (setq closed-list (append closed-list (list current-node)))
	   (setq closed-list (remove nil closed-list))
	   ;; stop if goal is found and return complete path
	   (if (equal (name current-node) (name end-node))
	       (return-from ucs-other (find-path current-node)))

	   ;; explore possible children nodes in row-cols left, right, bottom, and top 
	   ;; and generate those children
	   (setq children (expand-node graph current-node children))
	   ;; loop through the children nodes
	   (dolist (child children)
	     ;; check child is not in the closed list
	     (if (not (child-in-list? closed-list child))
		 (progn
		   ;; calculate total heuristic value
		   (setf (g child) (+ (g current-node) (g child)))
		   (setf (f child) (g child))
		   ;; add child to opened-list, if already present, check heuristic and 
		   ;; keep the child with least heuristic function value
		   (setq opened-list (add-child-opened-list opened-list child)) )))
	  ))))


;;; defining function for astar search algorithm
(defun astar-other (graph start-node end-node)
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
	   (if (equal (name current-node) (name end-node))
	       (return-from astar-other (find-path current-node)))

	   ;; explore possible children nodes in row-cols left, right, bottom, and top   
	   ;; and generate those children
	   (setq children (expand-node graph current-node children))
	   (setq children (remove nil children))
	   
	   ;; loop through the children nodes
	   (dolist (child children)
	     ;; check child is not in the closed list
	     (if (not (child-in-list? closed-list child))
		 (progn
		   (setf (g child) (+ (g current-node) 1))
		   ;; calculating manhattan heuristic function
		   (setf (h child) (heuristic child))
		   ;; calculate total heuristic value
		   (setf (f child) (+ (g child) (h child)))
		   ;; add child to opened-list, if already present, check heuristic and 
		   ;; keep the child with least heuristic function value
		   (setq opened-list (add-child-opened-list opened-list child)) )))
	  ))))

;;; function for checking the least heuristic value f in the opened list
(defun least-heuristic (opened-list current-node)
  (dolist (ele opened-list)
    (if (< (f ele) (f current-node)) (setq current-node ele)))
  current-node)
    
(defun heuristic (child)
  ;; ; list of heuristic as calculated in the graph represensation shown in write-up
  (let ((h-list (list 999999999999 1000 1000 1100 600 800 700 800 700 600 200 300 200 300 300 300 0 )))
    (return-from heuristic (elt h-list (name child)))))

;;;function to find the final path
(defun find-path (current-node)
  (let ((path (list nil)) (current current-node)) ; create a variable to store path
    (loop while (not (null current))
       do (progn (setq path (append path (list (name current))))
	    ;; store current node and all its successive parent nodes for
	    ;; the complete path
		 (setq current (parent current))))
    (setq path (reverse path)) ; reverse the path so that it's start-node to end-node
    (setq path (remove nil path))
    ;; return the complete path for astar search
    (return-from find-path path)))


;;; function to check of child is in the search-list
(defun child-in-list? (search-list child)
  (if (not (null search-list))
      (dolist (ele search-list)
	(if (equal (name child) (name ele))
	    (return-from child-in-list? T)))) ; returns T if child id found
      (return-from child-in-list? nil)) ; nil if child wasn't found

;;; function to add the possible children to the opened-list
(defun add-child-opened-list (opened-list child)
  (if (or (null opened-list) (equal opened-list (list nil)))
      (progn
	(setq opened-list (append opened-list (list child))) 
	(return-from add-child-opened-list (remove nil opened-list))))
  ;; check if node already present in opened list with greater
  ;; heuristic, replace
  (dolist (opened-node opened-list)
    (if (equal (name child) (name opened-node))
	(if (> (g opened-node) (g child))
	    (progn
	      (setq opened-list (remove opened-node opened-list))
	      (setq opened-list (append opened-list (list child)))
	      (return-from add-child-opened-list (remove nil opened-list)))
	    (return-from add-child-opened-list opened-list))))
  (setq opened-list (append opened-list (list child)))
  (return-from add-child-opened-list (remove nil opened-list)))