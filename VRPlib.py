from math import ceil
from scipy.cluster.vq import kmeans2, whiten
from TSPlib import nearest_neighbor, savings, greedy_two_opt

def CFRS(k, coordinates, d, plt):
    '''
    Constructs a VRP solution using cluster-first route-second (CFRS)
    heuristic.
    '''
    # CLUSTERING PHASE - Clusters the nodes based on their proximity to  
    # each other via k-means method, which uses Euclidean distance as
    # the proximity measure).
    x, y = kmeans2(whiten(coordinates), k, iter = 100)  
    plt.scatter(coordinates[:,0], coordinates[:,1], c = y)
    plt.axis('off')
    #plt.show()
    plt.savefig("clusters.png")
    
    depot = 0 # the depot node
    clusters = [{depot} for i in range(k)] # add the depot to all clusters
    
    # Assign the nodes to their respective clusters
    for i, label in enumerate(y):
        if i != depot:
            clusters[label].add(i)    
    print(clusters)
    
    # ROUTING PHASE - Constructs a list of TSP tours based on the clusters
    vrp_solution = [] # a list of TSP tours 
    total_length = 0 # total length of the TSP tours
    
    # Iterate over the clusters, and construct a TSP tour for each cluster
    for cluster in clusters:
        # Construct a TSP tour with your choice of construction method
        tour, tour_length = nearest_neighbor(cluster, depot, d)
        # Add the new tour to vrp_solution
        vrp_solution.append(tour)
        # Add the length of the new TSP tour to total_length
        total_length += tour_length
        
    # Round the result to 2 decimals to avoid floating point 
    # representation errors       
    total_length = round(total_length, 2)
    
    # Return the resulting VRP solution and its total length as a tuple
    return vrp_solution, total_length



def RFCS(k, nodes, d):
    '''
    Constructs a VRP solution using route-first cluster-second 
    (RFCS) heuristic.
    '''
    depot = 0 # the depot node
    n_max = ceil((len(nodes)-1)/k) # vehicle capacity
    
    # ROUTING PHASE - Constructs a TSP tour over all nodes with
    # your choice of construction method
    #tour, tour_length = nearest_neighbor(nodes, depot, d)
    tour, tour_length = savings(nodes, depot, d)
    # CLUSTERING PHASE - Iterates over the nodes in the TSP tour and
    # splits it into smaller tours each containing at most n_max nodes
    vrp_solution = []
    index = 1
    for i in range(k):
        current_tour = [depot]
        current_nodes = 0
        while current_nodes < n_max and tour[index] != depot:
            current_tour.append(tour[index])
            current_nodes += 1
            index += 1
        current_tour.append(depot)
        vrp_solution.append(current_tour)
        
    # Calculate the total length of the resulting tours
    total_length = 0
    for route in vrp_solution:
        for i in range(len(route)-1):
            total_length += d[route[i]][route[i+1]]
    
    # Round the result to 2 decimals to avoid floating point 
    # representation errors       
    total_length = round(total_length, 2)
    
    # Return the resulting VRP solution and its total length as a tuple
    return vrp_solution, total_length



def VRP_greedy_two_opt(vrp_solution, d):
    '''
    Greedily improves a given VRP solution using the 2-opt algorithm.
    It is a general function which can find solution for both asymmetric and symmetric matrices
    for a given VRP solution and a distance matrix. 
    '''
    greedy_vrp_solution = [] # to store vrp route data
    greedy_total_length = 0 # to count total length
    for tour in vrp_solution: # iterates over improved_sol that we found in 2-exchange algorithm 
        tour_length = sum(d[tour[i]][tour[i+1]] for i in range(len(tour)-1)) # computes total length of current tour
        new_tour, new_tour_length = greedy_two_opt(tour, tour_length, d) # I added greedy_two_opt to TSPlib
        greedy_vrp_solution.append(new_tour) # adds data to greedy_vrp_solution
        greedy_total_length += new_tour_length # counts total length
    return greedy_vrp_solution, round(greedy_total_length, 2)




def VRP_savings(nodes, origin, d, n_veh, demand_list, vehicle_capacity):
    '''
    Constructs a VRP solution using the savings method for a given set/list of 
    nodes, their pairwise distances-d, the origin, number of vehicle, demand list and the vehicle capacity.
    '''
    ans = [] # Creating an empty list to store solution data
    for iteration in range(n_veh): # every iteration is for finding the vehicle's route and the tour length of the route
            # Set of customer nodes (i.e. nodes other than the origin)
            customers = {i for i in nodes if i != origin}

            # Initialize out-and-back tours from the origin to every other node
            tours = {(i,i): [origin, i, origin] for i in customers}

            # Compute savings
            savings = {(i, j): round(d[i][origin] + d[origin][j] - d[i][j], 2) 
                       for i in customers for j in customers if j != i}

            # Define a priority queue dictionary to get a pair of nodes (i,j) which yields
            # the maximum savings
            #pq = pqdict(savings, reverse = True)
            sorted_savings = sorted(savings.items(), key=lambda item: item[1]) # the savings in ASC order
            # Merge subtours until obtaining a TSP tour
            break_while = False # to control while loop
            while len(tours) > 1 and not break_while: #if you have only 1 tour left then its last vehicle's tour no need to check
                A = sorted_savings.pop() # pops the maximum savings(because sorted_savings in ASC order) and stores it as a variable 
                # A = (i, j) --> biggest saving
                i = A[0][0] # i
                j = A[0][1] # j
      
                break_outer = False  # Outer loop

                for t1 in tours: # iterate all tours 
                    for t2 in tours.keys()-{t1}: # iterate over all tours except t1
                        sum_= 0
                        for x in (tours[t1][:-1] + tours[t2][1:]): # iterates over the tour if merge happens 
                            sum_ += demand_list[x] # to find capacity will if merge happens
                        if sum_ > vehicle_capacity: # if used capacity > vehicle capacity
                            break_while = True  
                            break
                        if t1[1] == i and t2[0] == j:  # checks the constraint 𝑖 is the last node in its subtour and 𝑗 is the first node in its subtour (i.e., (𝑖,𝑜) and (𝑜,𝑗) are in the subtours containing 𝑖 and 𝑗, respectively)                     
                            tours[(t1[0], t2[1])] = tours[t1][:-1] + tours[t2][1:] # merging the tours
                            del tours[t1], tours[t2] # delete the tours (e.g. 0-1-0, 0-2-0 merged 0-1-2-0. now we have to delete 0-1-0 and 0-2-0)
                            break_outer = True 
                            break
                    if break_outer:
                        break 

            x = tours.popitem() # delete the tour that achieved its capacity or that will exceed its capacity if one more merge happens
            
            ans.append(x[1]) # append the value of that tour to ans list
            for i in ans[iteration][1:]: # delete the nodes that used 
                for j in nodes:
                    if i == j:
                        nodes.remove(i)
                        
    # compute the answer's length
    ans_len = 0 
    for r in ans:
        for i in range(len(r)-1):
            ans_len += d[r[i]][r[i+1]]
            
    return ans, round(ans_len, 2) # return the ans and ans_len


def VRP_two_exchange(vrp_solution, d):
    '''
    Improves a given VRP solution using the 2-exchange algorithm.
    It is a general function which can solve asymmetric and symmetric matrices
    for a given VRP solution and distance matrix. 
    '''
    
    def swapList(sl,pos1,pos2):
        '''
        Swaps 2 list element according to their index.
        The list, and indexes of 2 item that will be swapped must be given. 
        '''
        n = len(sl) # n --> length of the list
        
        # Swapping 
        temp = sl[pos1]
        sl[pos1] = sl[pos2]
        sl[pos2] = temp
        return sl  
    
    def sol_leng(list, d):
        '''
        gives the route length of a list.
        a list and a distance matrix must be given
        '''
        sol_len= 0
        for i in range(len(list)-1):
            sol_len += d[list[i]][list[i+1]]
        return sol_len
     
    improved_sol = [] # the list that we will be store data of the solution
    improved_sol_length = 0 # with that var we keep track of the length of the soloution
    for r in range(len(vrp_solution)): # iterates the indexes in vrp_solution that given
        sol = list(vrp_solution[r]) # creating the list to compare
        sol_len = sol_leng(sol, d) # giving the length value to compare

        for i in range(len(sol)): # iterating the indexes of the sol
            if sol[i] != 0: # if sol is not the origin node
                #print(sol)
                #print(sol_len)
                for j in range(len(sol)): # iterating the indexes of the sol
                    if sol[i] != sol[j] and sol[j] != 0: # if they are not the same or the origin node
                        # Making the exchanged list 
                        temp = list(sol) 
                        temp = list(swapList(temp, i, j))
                        temp_len = sol_leng(temp, d)
                        #print("swapping", i," and ", j)
                        #print("temp: ", temp, "temp len: ", temp_len)
                        # To control if we have a better solution after exchange
                        if temp_len < sol_len: # If we have a better solution via exchange we will make the exchange
                            #print("solution improved")
                            sol_len = temp_len
                            sol = list(temp)
        improved_sol.append(sol) # append the improved solution to improved_sol
        improved_sol_length += sol_len # add the length of the tour 
            
    return improved_sol, improved_sol_length

