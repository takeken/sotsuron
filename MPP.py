from __future__ import print_function
import cplex
import sys
import itertools 
import math









##################################

#traffic demand
b_t = 100
M_t = 1
rho_t = 0.9
T =[b_t,M_t,rho_t]

#bandwidth requirement
b = T[0]
#number of failures
M = T[1]
#partial protection requirement
rho = T[2]


#about paths
#the number of paths
N = 3
#set of paths
P =[a for a in range(N)]
#the number of hops
hop = [1,1,1]
#modulation efficisncy
eta = [20,20,20]


#survive paths
not_F = list(itertools.combinations(P,(N-M)))




#variables
#the number of slots of k-th path
B_k = []



#############
def setupproblem(c):
  
  c.objective.set_sense(c.objective.sense.minimize)
  
  # assignment variables:B_k

  for a in range(N):
    varname = "B_" + str(a)

    B_k.append(varname)





###############


  #objective
    
  c.variables.add(names=B_k,
    lb=[1]*len(B_k),
    ub=[320]*len(B_k),
    types=["I"]*len(B_k),
    obj = hop) #hopに変える


#constraints

  #\sum_{k in K}\eta_k*B_k >= b
  thevars = []
  thecoefs = []
  for a in P:
    thevars.append(B_k[a])
    thecoefs.append(eta[a])
    
  c.linear_constraints.add(
    lin_expr=[cplex.SparsePair(thevars, thecoefs)],
    senses=["G"],
    rhs=[b])
    
  #\sum _{k\in K:k\not\in F}eta_k*B_k >= rho*b
  for not_f in not_F:
    thevars = []
    thecoefs = []
    for a in list(not_f):
      thevars.append(B_k[a])
      thecoefs.append(eta[a])
      
    c.linear_constraints.add(
      lin_expr=[cplex.SparsePair(thevars, thecoefs)],
      senses=["G"],
      rhs=[rho*b])
    
    
    
    

#########


def assignment():
  
  c = cplex.Cplex()
  
  setupproblem(c)
  
  c.write("MPP.lp")
  
  c.solve()
  
  
  sol = c.solution
  
  print()
  
  if sol.is_primal_feasible():
    
    a = int(sol.get_objective_value())
    print("B_k = {0}" .format(
      sol.get_values(B_k)))
    print()
    b = sol.get_values(B_k)
  
  
  else:
    print("No solution available.")
  return a,b
  
  











if __name__ == "__main__":
  
  
  print("required slots = ",assignment())
  print()
  
