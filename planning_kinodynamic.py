import matplotlib.pyplot as plt
import numpy as np
import math

import warnings
import time



class planning_kino:
    """Implementing Kinodynamic planning algorithms"""

    def __init__(self, iterations, domain, num_of_obstacles):
        self.q_init = []
        self.q_goal = []
        self.radius_q_goal  = 0.6
        self.iterations = iterations
        self.delta = 1
        self.domain = domain
        self.q_list = []
        self.circles = []
        self.rectangles = []
        self.num_of_obstacles = num_of_obstacles
        self.q_prev_list = [[]]
        self.cost_list = [0]
        self.neighborhood_radius = 3
        self.q_best = []
        self.cost_min = 0
        self.cost_to_goal = float('inf')
        self.q_centre = []
        self.u_bounds = []
        self.u_new = []
        self.Te = 1
        #self.cost_prev_list = []
    
    def generate_random_config(self):
        """Generate a random position in the domain"""
        self.q_rand = [self.domain * np.random.rand(), self.domain * np.random.rand(), np.random.uniform(-math.pi, math.pi)]
        return self.q_rand
    
    def set_q_init(self,q_init):
        """Set a deterministic init vertex (it must be consistent with the positions of the obstacles)"""    
        if not self.circles and not self.rectangles:
            self.create_random_obstacle()

        #print(not self.check_vertex_in_any_circles(q_init))
        if not self.check_vertex_in_any_circles(q_init):
            self.q_init = q_init;
        else:
            warnings.warn('Your q_init is inside an obstacle, another random value  will be chosen')
            print("Your q_init is inside an obstacle, another random value will be chosen")
        
    def set_q_goal(self,q_goal):
        """Set a deterministic goal vertex (it must be consistent with the positions of the obstacles)"""
        if not self.circles and not self.rectangles:
            self.create_random_obstacle()    
        
        if not self.check_vertex_in_any_circles(q_goal):
            self.q_goal = q_goal;
        else:
            warnings.warn('Your q_goal is inside an obstacle, another random value will be chosen')
            print("Your q_goal is inside an obstacle, another random value will be chosen")       

    def set_u_bounds(self,u_bounds):
        """Set control input lower and upper bounds"""
        self.u_bounds = np.array(u_bounds)

    def find_nearest_vertex(self):
        """Find the nearest vertex from a random position in q_list"""
        dist_list = []
        for vertex in self.q_list:
            distance = math.dist(vertex, self.q_rand)
            dist_list.append(distance)
        nearest_dist = min(dist_list)
        nearest_index = dist_list.index(nearest_dist)
        self.q_near = self.q_list[nearest_index]
        return self.q_near

    def generate_new_config(self):
        """Generate a new vertex"""

        print("Depreciated")
        distance = math.dist(self.q_rand, self.q_near)
        x_diff = self.q_rand[0] - self.q_near[0]
        y_diff = self.q_rand[1] - self.q_near[1]
        q_new_x = self.q_near[0] + (self.delta / distance) * x_diff
        q_new_y = self.q_near[1] + (self.delta / distance) * y_diff

        #q_new_x = self.q_near[0] + 0.2 * x_diff
        #q_new_y = self.q_near[1] + 0.2 * y_diff
        self.q_new = [q_new_x, q_new_y]
        return self.q_new

    def create_random_obstacle(self):
        """Create a random circular obstacle"""
        while self.num_of_obstacles != 0:
            center_x = np.random.randint(1, 100)
            center_y = np.random.randint(1, 100)
            center = [center_x, center_y]
            radius = np.random.randint(1, 10)
            self.circles.append([center, radius])
            #print(self.circles)
            self.num_of_obstacles -= 1
    
    def set_circle_obstacles(self,circles):
        """Add circular obstacles """
        for circle in circles:
            self.circles.append(circle)

    def set_rectangle_obstacles(self,rectangles):
        """Add rectangles obstacles (aligned with x-y axis) """
        # rectangle = [left low corner position, rectangle height, rectangle width] 
        for rectangle in rectangles:
            self.rectangles.append(rectangle)


    def check_vertex_in_circle(self, vertex, circle):
        """Check if the vertex lies inside or on the circle"""
        distance = math.dist(circle[0], vertex[0:2]) # distance to vertex position (not all state)
        if distance <= circle[1]:
            return True
        return False
    
    def check_vertex_in_any_circles(self,vertex):
        """Check if the vertex lies inside or on one of the circle"""
        for circle in self.circles:
            if self.check_vertex_in_circle(vertex, circle):
                return True
        return False
    
    def check_vertex_in_rectangle(self, vertex, rectangle):
        """Check if the vertex lies inside or on the circle"""
        if vertex[0]>=rectangle[0][0] and vertex[0]<=rectangle[0][0]+ rectangle[1] and vertex[1]>=rectangle[0][1] and vertex[1]<=rectangle[0][1]+ rectangle[2]:
            return True
        return False

    def find_t_segment(self, point_c, point1, point2):
        """Calculate u required to check for path collision"""
        """ point 1 and point 2 are the two edges/endpoints of the segment """

        x_delta = point2[0] - point1[0]
        y_delta = point2[1] - point1[1]
        u = ((point_c[0] - point1[0]) * x_delta + (point_c[1] - point1[1]) * y_delta) / (x_delta * x_delta + y_delta * y_delta)
        return u

    def check_path_collision(self, circle, point1, point2):
        """Check if the path from a vertex to another vertex intersects with a circle"""
        x_delta = point2[0] - point1[0]
        y_delta = point2[1] - point1[1]

        """ Determine first the closest point from the segment to another point (circle center) """
        t = self.find_t_segment(circle[0], point1, point2)
       
        if t < 0:
            closest_point = point1[0:2]
        elif t > 1:
            closest_point = point2[0:2]
        else:
            closest_point = (point1[0] + t * x_delta, point1[1] + t * y_delta)
        
        distance = math.dist(closest_point, circle[0])
        if distance <= circle[1]:
            return True
        return False
    
    def check_path_collision_rectangle(self, rectangle, point1, point2):
        """Check if the path from a vertex to another vertex intersects with a rectangle"""

        # allready done
        # for rectangle in self.rectangles:
        #     if self.check_vertex_in_rectangle(point1, rectangle) or self.check_vertex_in_rectangle(point2, rectangle):
        #         return True

        #Check if there are collision from the equality between
        #  segment equation and one of the 4 rectangles edges
        for rectangle in self.rectangles:
            x_rect = rectangle[0][0]
            y_rect = rectangle[0][1]
            height_rect = rectangle[1]
            width_rect = rectangle[2]
            delta_x = point2[0] - point1[0]
            delta_y = point2[1] - point1[1]

            list_M = np.array([[[0, delta_x],[ height_rect, delta_y]],[[0, delta_x],[height_rect, delta_y]], [[width_rect, delta_x],[ 0, delta_y]], [[width_rect, delta_x],[ 0, delta_y]]])
            
            list_vect_diff = np.array([[point1[0]-x_rect , point1[1]-y_rect],[point1[0]-x_rect-width_rect, point1[1]-y_rect],[point1[0]-x_rect, point1[1]-y_rect],[point1[0]-x_rect, point1[1]-y_rect-height_rect]])

            for i in range(len(list_M)):
                #M = [[0, delta_x],[ height_rect, delta_y]]
                #det_M = -delta_x*height_rect
                M = list_M[i]
                det_M = M[0][0]*M[1][1] - M[1][0]*M[0][1]

                if det_M != 0:
                    ##
                    #vect_diff = np.array([point1[0]-x_rect , point1[1]-y_rect])
                    vect_diff = list_vect_diff[i]
                    M_inverse = 1/det_M*np.array([[M[1][1],-M[0][1]], [M[0][0],-M[1][0]]]) #explicit formula

                    t1_vect = M_inverse @ vect_diff
                    #print("t1_vect",t1_vect)


                    if t1_vect[0] >= 0 and t1_vect[0] <= 1 and  -t1_vect[1] >= 0 and -t1_vect[1] <= 1:
                        # x_intersect = point1[0]-t1_vect[1]*(point2[0]-point1[0])
                        # y_intersect = point1[1]-t1_vect[1]*(point2[1]-point1[1])
                        # print("Intersection at : (",x_intersect,",",y_intersect,")")
                        return True
            
                else: #det_M == 0
                    #print("Special case to test")

                    print("Still to implement this rare case det_M == 0")
        

        return False

    def detect_point_collision(self, rectangle, point1, point2):
        """Detect and return the collision points of the path from a 
        vertex to another vertex with a rectangle"""

        for rectangle in self.rectangles:
            x_rect = rectangle[0][0]
            y_rect = rectangle[0][1]
            height_rect = rectangle[1]
            width_rect = rectangle[2]
            delta_x = point2[0] - point1[0]
            delta_y = point2[1] - point1[1]
            M1 = [[0, delta_x],[ height_rect, delta_y]]
            det_M1 = -delta_x*height_rect

            if det_M1 != 0:
                vect_diff = np.array([point1[0]-x_rect , point1[1]-y_rect])
                t1_vect = np.inverse(M1)*vect_diff
                M1_inverse = [] #explicit formula

                if t1_vect[0] >= 0 and t1_vect[0] <= 1 and  -t1_vect[1] >= 0 and -t1_vect[1] <= 1:
                    x_intersect = point1[0]-t1_vect[1]*(point2[0]-point1[0])
                    y_intersect = point1[1]-t1_vect[1]*(point2[1]-point1[1])
                    print("Intersection at : (",x_intersect,",",y_intersect,")")
                    return [x_intersect,y_intersect]

        print("To finish")
        #https://stackoverflow.com/questions/2368211/line-rectangle-collision-detection

        return []


    def check_collision(self, new_vertex, q_origin):
        """Check if there is any collision"""
        """ Under the assumption that q_origin is a valid vertex not in an obstacle """

        for circle in self.circles:
            if self.check_vertex_in_circle(new_vertex, circle) or self.check_path_collision(circle, q_origin, new_vertex):
                return True
        
        for rectangle in self.rectangles:

            #test_vertex_in_rect = self.check_vertex_in_rectangle(new_vertex, rectangle)
            #test_path_in_rect = self.check_path_collision_rectangle(rectangle, q_origin, new_vertex)
            if self.check_vertex_in_rectangle(new_vertex, rectangle) or self.check_path_collision_rectangle(rectangle, q_origin, new_vertex):
                return True

        return False

    def check_collision_free_path(self):
        """Check for a collision free path from a new vertex to the goal"""

        #detect reachable node in the goal area

        for circle in self.circles:
            if self.check_path_collision(circle, self.q_new, self.q_goal):
                return False
        return True

    def generate_random_start(self):
        """Generate a random start location"""
        self.q_init = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.check_vertex_in_circle(self.q_init, circle):
                self.q_init = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_init
    
    def generate_random_goal(self):
        """Generate a random goal location"""
        self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.check_vertex_in_circle(self.q_goal, circle):
                self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_goal

    def trace_optimal_path(self,ax):
        #Trace the current optimal path

        current = self.q_list[-1]
        previous = self.q_prev_list[-1]

        #x2 = [current[0], self.q_goal[0]]
        #y2 = [current[1], self.q_goal[1]]
        #ax.plot(x2, y2, color='red')
        
        while True:
            if previous == []:
                break
            x3 = [current[0], previous[0]]
            y3 = [current[1], previous[1]]
            ax.plot(x3, y3, color='red')
            plt.pause(0.0001)
            current = previous
            if current in self.q_list:
                index = self.q_list.index(current)
            previous = self.q_prev_list[index]


    def check_vertices_around(self):
        """ Find neighbors and the one with cheapest costs """

        q_neighbors = []
        min_cost = float('inf')
        for q in self.q_list:
            
            dist_q = math.dist(q,self.q_new)
            if dist_q <= self.neighborhood_radius:
                
                new_cost = self.cost_list[self.q_list.index(q)] + dist_q
                q_neighbors.append(q)

                if new_cost < min_cost:

                    min_cost = new_cost
                    q_best = q
            
        return q_best,q_neighbors
    
    def update_links(self,q_neighbors,lines): #
        """ Optimize the path cost with the new vertex"""
        """ Rewire tree to shorten edges if possible """

        for q in q_neighbors:

            current_cost_neighbor = self.cost_list[self.q_list.index(q)]
            new_cost_neighbor = self.cost_list[self.q_list.index(self.q_new)] + math.dist(q,self.q_new)

            if current_cost_neighbor > new_cost_neighbor and not self.check_collision(q, self.q_new): # and check no collision path
                #print("Worth to rewire with q_new = ",self.q_new)
                #print("q=",q)      
                #print("lines:",lines[self.q_list.index(q)-1][0].get_data())

                self.cost_list[self.q_list.index(q)] = new_cost_neighbor
                self.q_prev_list[self.q_list.index(q)] = self.q_new

                x1 = [q[0], self.q_new[0]]
                y1 = [q[1], self.q_new[1]]

                # Change the old edge by the new one
                lines[self.q_list.index(q)-1][0].set_data([x1, y1])
                #lines[self.q_list.index(q)-1][0].set(color='green', linewidth=2.5)
               
                # Change the cost of the children nodes of q
                self.update_cost_path_from_q(q,self.q_list.index(q))

                #arti = ax.plot(x1, y1, color='green')
                #ax.lines.remove(lines[self.q_list.index(q)])
                #ax.lines.append(ax.plot(x1, y1, color='green'))
                #ax.add_artist(arti)
                #ax.draw_artist()  

    def update_cost_path_from_q(self,q,index_q):
        """ Update the cost of all vertex in a path that start from q"""
        """ in a recursive way """

        q_children_index = []
        q_children = []
        for index_child, q_prev in enumerate(self.q_prev_list):
            if q_prev == q:
                q_children_index.append(index_child)
                q_child = self.q_list[index_child]
                q_children.append(q_child)

                self.cost_list[index_child] = self.cost_list[index_q] + math.dist(q,self.q_list[index_child])

                self.update_cost_path_from_q(q_child,index_child)

        #= self.q_prev_list.index(q)

    def random_input_propagation(self,q_near):
        """Generate a random input inside the bounds and propagate dynamic"""

        u_random = np.random.uniform(self.u_bounds[:,0], self.u_bounds[:,1])

        q_new = self.RK4(q_near,u_random,self.Te)


        random_time_steps = 2
        # U_random = 
        #q_list = self.propagate_dynamic(X0,Uk,random_time_steps,self.Te,4)
        #q_new = q_list[-1]

        return q_new,u_random


    def f_dyn(self,t,Xk,uk):
        # Xk = [xk,yk,thetak], state at k
        # uk = [vk,wk], command at k
        # t : time
        
        dXk = [ uk[0]*math.cos(Xk[2]), uk[0]*math.sin(Xk[2]), uk[1] ]

        return dXk

    def Euler_explicit(self,Xk,uk,Te):

        dXk = self.f_dyn(0,Xk,uk)

        #Xk_1 = Xk + Te*dXk
        Xk_1 = [Xk[i] + Te*dXk[i] for i in range(len(Xk)) ]

        Xk_1[2] = self.normalize_angle(Xk_1[2])
        return Xk_1
    
    def RK2(self,Xk,uk,Te):

        k_1 = self.f_dyn(0,Xk,uk)
        Xk_12 = [Xk[i] + Te/2*k_1[i] for i in range(len(Xk)) ]
        #k_2 = self.f_dyn(0,Xk + Te/2*k_1,uk)
        k_2 = self.f_dyn(0,Xk_12,uk)

        #Xk_1 = Xk + Te*k_2
        Xk_1 = [Xk[i] + Te*k_2[i] for i in range(len(Xk)) ]

        Xk_1[2] = self.normalize_angle(Xk_1[2])
        return Xk_1
    
    def RK4(self,Xk,uk,Te):
        # Xk : state at k
        # uk : command at k
        # Te : sampling period
        # Xk_1 : state at k+1 
        k_1 = self.f_dyn(0,Xk,uk)
        Xk_12 = [Xk[i] + Te/2*k_1[i] for i in range(len(Xk)) ]
        k_2 = self.f_dyn(0,Xk_12,uk)
        Xk_23 = [Xk[i] + Te/2*k_2[i] for i in range(len(Xk)) ]
        k_3 = self.f_dyn(0,Xk_23,uk)
        Xk_34 = [Xk[i] + Te*k_3[i] for i in range(len(Xk)) ]
        k_4 = self.f_dyn(0,Xk_34,uk)

        #Xk_1 = Xk + Te/6*(k_1 + 2*k_2 + 2*k_3 + k_4)
        Xk_1 = [Xk[i] + Te/6*(k_1[i] + 2*k_2[i] + 2*k_3[i] + k_4[i]) for i in range(len(Xk)) ]

        Xk_1[2] = self.normalize_angle(Xk_1[2])
        return Xk_1
    
    def propagate_dynamic(self,X0,Uk,N_h,Te,flag_discretization):
        """ Propagate the dynamic """
        # Uk : list of input vector
        # Nh : horizon length

        Xk_horizon = [X0]

        Xk = X0

        if flag_discretization == 1:
            for k in range(N_h):

                Xk = self.Euler_explicit(Xk,Uk[k],Te)
                #Xk_horizon = [Xk_horizon, Xk]
                Xk_horizon.append(Xk) 
        elif flag_discretization ==2:
            for k in range(N_h):

                Xk = self.RK2(Xk,Uk[k],Te)
                #Xk_horizon = [Xk_horizon, Xk]
                Xk_horizon.append(Xk) 
        else: #flag_discretization == 4
            for k in range(N_h):

                Xk = self.RK4(Xk,Uk[k],Te)
                #Xk_horizon = [Xk_horizon, Xk]
                Xk_horizon.append(Xk) 
        
        return Xk_horizon

    def normalize_angle(self,angle):
        """ Normalize phi to be between -pi and pi"""
        #angle can also be a vector of angles 

        normalized_angle = ((angle + math.pi) % (2*math.pi)) - math.pi
        return normalized_angle

    def init_map_plot(self,ax):
        """ Init the map environnement, init and goal poses, obstacles and the plot"""
        ax.set_xlim(0, self.domain)
        ax.set_ylim(0, self.domain)

        if not self.circles and not self.rectangles:
            self.create_random_obstacle()

        if not self.q_init:
            self.q_init = self.generate_random_start()
        
        self.q_list.append(self.q_init)

        if not self.q_goal:
            self.q_goal = self.generate_random_goal()

        ax.plot(self.q_init[0], self.q_init[1], 'o', color='green')

        ax.plot(self.q_goal[0], self.q_goal[1], 'o', color='orange')
        circle_goal = plt.Circle(self.q_goal, self.radius_q_goal, color='orange', fill=False) #True
        ax.add_artist(circle_goal)

        for circle in self.circles:
            circle = plt.Circle(circle[0], circle[1], color='black', fill=False) #True
            ax.add_artist(circle)
        
        for rectangle in self.rectangles:
            rectangle = plt.Rectangle(rectangle[0], rectangle[1], rectangle[2], color='black', fill=True,alpha=0.1) #True
            #rectangle = plt.Rectangle(rectangle[0], rectangle[1], rectangle[2], color='black', fill=False) 
            ax.add_artist(rectangle)     

    def check_in_goal_area(self,q_new):
        """ Check if a vertew q_new is in the goal area"""
        if math.dist(q_new[0:2],self.q_goal[0:2]) <= self.radius_q_goal:
            return True
        else: 
            return False

    def evaluate_new_cost(self,q_near,q_new,u_new):

        cost = self.cost_list[self.q_list.index(q_near)] + math.dist(q_near,q_new) + 0.1*np.linalg.norm(u_new) 
        #q_near[0:2] q_new[0:2]

        return cost

    def plot_result_rrt_kino_random(self):
        """Plot the Kinodynamic RRT"""
        """ Random u / Best u"""

        t_start = time.time()
        f, ax  = plt.subplots()
        plt.ion()

        ax.set_title("Kinodynamic RRT with Obstacles")
        
        self.init_map_plot(ax)
        plt.show()
        
        for iteration in range(self.iterations):
            self.q_rand = self.generate_random_config()
            self.q_near = self.find_nearest_vertex()
            
            #self.q_new = self.generate_new_config()
            self.q_new, self.u_new = self.random_input_propagation(self.q_near)

            #log
            #x1 = [self.q_near[0], self.q_new[0]]
            #y1 = [self.q_near[1], self.q_new[1]]
            #ax.plot(x1, y1, marker="o",color='olive',linewidth=1, markersize=4) 
            #linestyle='dashed',
            #print("Iteration:",iteration,",",self.check_collision(self.q_new, self.q_near))

            if not self.check_collision(self.q_new, self.q_near):
                self.q_list.append(self.q_new)
                #self.cost_list.append(self.cost_list[self.q_list.index(self.q_near)] + math.dist(self.q_near,self.q_new))
                self.cost_list.append(self.evaluate_new_cost(self.q_near,self.q_new, self.u_new))
                self.q_prev_list.append(self.q_near)
                x1 = [self.q_near[0], self.q_new[0]]
                y1 = [self.q_near[1], self.q_new[1]]
                # plot new edges
                ax.plot(x1, y1, color='blue')
                plt.pause(0.00001)

                if self.check_in_goal_area(self.q_new):
                    # If node in goal then stop

                    #self.cost_list.append(self.cost_list[self.q_list.index(self.q_new)] + math.dist(self.q_new,self.q_goal))
                    # self.cost_list.append(self.evaluate_new_cost(self.q_near,self.q_new))
                    
                    self.q_list.append(self.q_goal)
                    self.q_prev_list.append(self.q_new)

                    plt.pause(0.0001)
                    print("Number of iterations: ",iteration)
                    print("Cost: ",self.cost_list[-1])
                    break
                    
        self.trace_optimal_path(ax)

        t_end = time.time()
        print(f"Total runtime of RRT is {t_end - t_start} seconds")       
        #plt.show()
        plt.show(block=True)
        #print(self.q_prev_list)
        #print(self.q_list)
        #plt.pause(5)


    def plot_result_kino_star(self):
        """Plot the RR tree star *"""
        t_start = time.time()
        f, ax = plt.subplots()
        lines = []
        #plt.ion()

        ax.set_title("Kinodynamic Rapidly-Exploring Random Tree * with Obstacles")
        

        self.init_map_plot(ax)
        
        
        for iteration in range(self.iterations):
            self.q_rand = self.generate_random_config()
            self.q_near = self.find_nearest_vertex()

            self.q_new = self.generate_new_config()

            [self.q_best,q_neighbors] = self.check_vertices_around()

            if not self.check_collision(self.q_new,self.q_best):

                # Add the cost to reach self.q_new
                self.cost_list.append(self.cost_list[self.q_list.index(self.q_best)] + math.dist(self.q_best,self.q_new))
                self.q_list.append(self.q_new) # list of vertex
                self.q_prev_list.append(self.q_best) # list of parents vertex
                x1 = [self.q_best[0], self.q_new[0]]
                y1 = [self.q_best[1], self.q_new[1]]
                # plot new edges
                lines.append(ax.plot(x1, y1, color='blue'))
                #ax.plot(x1, y1, color='blue')
                plt.pause(0.0001)

                self.update_links(q_neighbors,lines)

                plt.pause(0.0001)    

                if self.check_collision_free_path():
                    # If free path to goal then stop
                    x2 = [self.q_new[0], self.q_goal[0]]
                    y2 = [self.q_new[1], self.q_goal[1]]
                    self.cost_list.append(self.cost_list[self.q_list.index(self.q_new)] + math.dist(self.q_new,self.q_goal))
                    ax.plot(x2, y2, color='blue')
                    plt.pause(0.0001)
                    print("Number of iterations: ",iteration)
                    print("First cost-to-goal: ",self.cost_list[-1]) # f-value
                    break

        self.q_list.append(self.q_goal)
        self.q_prev_list.append(self.q_new)
        self.cost_list.append(math.dist(self.q_new,self.q_goal))
        current = self.q_list[-1]
        previous = self.q_prev_list[-1]

        #Trace the optimal path
        while True:
            if previous == []:
                break
            x3 = [current[0], previous[0]]
            y3 = [current[1], previous[1]]
            ax.plot(x3, y3, color='red')
            plt.pause(0.0001)
            current = previous
            if current in self.q_list:
                index = self.q_list.index(current)
            previous = self.q_prev_list[index]

        t_end = time.time()
        print(f"Total runtime of RRT* is {t_end - t_start} seconds")    
        
        #plt.show()
        #plt.show(block=True)
        #plt.pause(15)
    
    def plot_result_rrt_kino_informed_star(self):
        """Plot the Kinodynamic Informed RR tree star *"""
        t_start = time.time()
        f, ax = plt.subplots()
        lines = []
        #plt.ion()

        ax.set_title("Kinodynamic Rapidly-Exploring Informed Random Tree * with Obstacles")
        
        self.init_map_plot(ax)
        
        for iteration in range(self.iterations):
            self.q_rand = self.generate_random_config()
            self.q_near = self.find_nearest_vertex()

            self.q_new = self.generate_new_config()

            [self.q_best,q_neighbors] = self.check_vertices_around()

            if not self.check_collision(self.q_new,self.q_best):

                # Add the cost to reach self.q_new
                self.cost_list.append(self.cost_list[self.q_list.index(self.q_best)] + math.dist(self.q_best,self.q_new))
                self.q_list.append(self.q_new) # list of vertex
                self.q_prev_list.append(self.q_best) # list of parents vertex
                x1 = [self.q_best[0], self.q_new[0]]
                y1 = [self.q_best[1], self.q_new[1]]
                # plot new edges
                lines.append(ax.plot(x1, y1, color='blue'))
                #ax.plot(x1, y1, color='blue')
                plt.pause(0.0001)

                self.update_links(q_neighbors,lines)

                plt.pause(0.0001)    
                
                # difference with RRT star
                if self.check_collision_free_path():
                    # If free path to goal then add this path (and stop)
                    #self.cost_list.append(self.cost_list[self.q_list.index(self.q_new)] + math.dist(self.q_new,self.q_goal))
                    
                    new_cost_to_goal = self.cost_list[self.q_list.index(self.q_new)] + math.dist(self.q_new,self.q_goal)

                    if new_cost_to_goal<self.cost_to_goal:
                        self.cost_to_goal = new_cost_to_goal

                        x2 = [self.q_new[0], self.q_goal[0]]
                        y2 = [self.q_new[1], self.q_goal[1]]
                        ax.plot(x2, y2, color='blue')
                        plt.pause(0.0001)        
                        #self.q_list.append(self.q_goal)
                        #self.q_prev_list.append(self.q_new)

                        print("Number of iterations: ",iteration)
                        print("Cost-to-goal: ",self.cost_to_goal)
                        #break
                        self.trace_optimal_path(ax)

        t_end = time.time()
        print(f"Total runtime of the program is {t_end - t_start} seconds")  
        #plt.show()
        plt.show(block=True)
        #plt.pause(15)




def main():
    
    # Kinodynamic motion planning
    # (Optimal) Kinodynamic Planning 

    #For repeatability
    np.random.seed(0)
    #np.random.seed(1)

    q_init = [1,5,0]
    q_goal = [17,15,0]
    #radius_q_goal = 1

    dist_min = math.dist(q_init[0:2],q_goal[0:2]) 
    
    Te = 0.1 # 0.05

    v_max = 8  # 0.22
    u_bounds = [[0,v_max],[-3,3]]
    N_min_step = dist_min/v_max/Te
    print(N_min_step)
    cost_min = dist_min + N_min_step*0.1*np.linalg.norm(v_max)
    print("Cost min: ",cost_min) #lower bound #without obstacle
    
    #u_bounds = [[-v_max,v_max],[-2.84,2.84]] # turtlebot Burger

    len_map = 20 # 20
    Niterations = 6000 # 1000 500
    kino = planning_kino(Niterations,len_map, 4)

    rectangles = [[[0,0],len_map,4],[[0,7],9,7],[[11,7],9,7],[[0,16],len_map,4],[[20,4],4,12]]
    circles = [[[14,7],5],[[7,15],5],[[14.5,13.5],1]]
    
    kino.set_circle_obstacles(circles)

    #kino.set_rectangle_obstacles(rectangles)

    kino.Te = Te
    kino.set_q_init(q_init) # 1,1
    kino.set_q_goal(q_goal)
    kino.set_u_bounds(u_bounds)
    kino.plot_result_rrt_kino_random()

    #np.random.seed(0)

# In the distance condition do we take into account the heading angle of the robot ?

# Save as a GIF 

# RRT* with vehicle dynamic
# LQR-RRT
# MPPI Model Predictive Path Integral 
    #rrt.plot_result_with_dynamics()

if __name__ == "__main__":
    main()