import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from ortools.linear_solver import pywraplp

#Formulacao de Carvalho
def Carvalho(G,n):
  solver = pywraplp.Solver.CreateSolver('SCIP')
  solver.SetTimeLimit(1800*1000) #Definindo máximo de tempo
  # vetor de cores
  collors = []
  for j in range(n):
      collors.append(solver.IntVar(0.0, 1.0, 'x' + str(j)))

  #Criação do MAP
  map = {}
  for v in range(n):
    for j in range(n):
      map[v,j]  = solver.IntVar(0.0, 1.0, 'y{}'.format((v,j)))


  #Criação da função objetiva
  solver.Maximize(solver.Sum(collors[i] for i in range(n)))

  #Sujeito a:

  #Tentando deixas as retrições na mesma ordem do artigo

  #3.2.2
  for v in range(n):
    solver.Add(solver.Sum(map[v,i] for i in range(n)) == 1)

  #3.2.3
  for j in range(n):
    for (u,v) in G.edges():
      solver.Add(map[v,j] + map[u,j] <= collors[j])


  #3.2.4
  for j in range(n):
      solver.Add(collors[j] <= solver.Sum(map[v,j] for v in range(n)))

  #3.2.5
  for c in range(n):
    for v in range(n):
      if len(list(G.neighbors(v))) == 0:
        solver.Add(map[v,c] <= collors[c])

  #3.2.6
  for c in range(n):
    for c_prime in range(c + 1, n):
      solver.Add(collors[c_prime] <= collors[c])


  #3.2.7
  for v in range(n):
    for c in range(n):
      for c_prime in range(c + 1, n):
        solver.Add(map[v,c_prime] <= solver.Sum(map[u,c] for u in G.neighbors(v)))


  #Solucionar
  final = solver.Solve()
  color_map = []

  if final == pywraplp.Solver.OPTIMAL:
    print('Solution:')
    print('Objective value =', solver.Objective().Value())

    for i in range(n):
        print("x{} = {}".format(i, collors[i].solution_value()) )

    for v in range(n):
      for i in range(n):
        if map[v,i].solution_value() == 1:
          print("vertice {} recebeu cor {}".format(v,i) )

  else:

      nx.draw(G, node_color=color_map, with_labels=True)
      print('The problem does not have an optimal solution.')

  print('\nAdvanced usage:')
  print('Problem solved in %f milliseconds' % solver.wall_time())
  print('Problem solved in %d iterations' % solver.iterations())
  print('Problem solved in %d branch-and-bound nodes' % solver.nodes())



def Rodrigues(G,n):
  solver = pywraplp.Solver.CreateSolver('SCIP') # Criação do Solver
  solver.SetTimeLimit(1800*1000) #Definindo o tempo maximo para 30mins

  #Vetor das cores
  collors = []
  for j in range(n):
    collors.append(solver.IntVar(0.0, 1.0, 'x' + str(j)))

  #Dicionario chave->valor
  map = {}
  for v in range(n):
    for j in range(n):
      map[v,j]  = solver.IntVar(0.0, 1.0, 'y{}'.format((v,j)))

  #Função Objetiva
  solver.Maximize(solver.Sum(collors[i] for i in range(n)))

  #Sujeito a:

  #----------------#
  # 3.1.2
  for v in range(n):
    solver.Add(solver.Sum(map[v,i] for i in range(n)) == 1)

  # 3.1.7
  for i in range(n-1):
    solver.Add(collors[i] >= collors[i+1])
  
  # 3.1.3
  for j in range(n):
    for (u,v) in G.edges():
      solver.Add(map[v,j] + map[u,j] <= collors[j])
  

  # 3.1.4
  for j in range(n):
    solver.Add(collors[j] <= solver.Sum(map[v,j] for v in range(n)))

 #-----------------#

  #3.1.6
  for v in range(n):
    if G.degree(v) == 0:
        solver.Add(map[v,0] == 1)

  # 3.1.5
  for v in range(n):
    for c in range(n):
      solver.Add(map[v, c] >= 1 - solver.Sum(map[v, d] for d in range(c)) - solver.Sum(map[u, c] for u in G.neighbors(v)))


  #Solucionar
  final = solver.Solve()
  if final == pywraplp.Solver.OPTIMAL:
    print('Solution:')
    print('Objective value =', solver.Objective().Value())

    for i in range(n):
      print("x{} = {}".format(i, collors[i].solution_value()) )

    color_map = []
    for v in range(n):
      for i in range(n):
        if map[v,i].solution_value() == 1:
          print("vertice {} recebeu cor {}".format(v,i) )
    nx.draw(G, node_color=color_map, with_labels=True)
    

  else:
      print('The problem does not have an optimal solution.')
      
  print('\nAdvanced usage:')
  print('Problem solved in %f milliseconds' % solver.wall_time())
  print('Problem solved in %d iterations' % solver.iterations())
  print('Problem solved in %d branch-and-bound nodes' % solver.nodes())



def main():
  #Tamanho e densidade
  tam = [10,13,15,18,20]
  dens = [0.5,0.8]

  for t in tam:
    for d in dens:
      G = nx.gnp_random_graph(t,d) #Criando a instancia do grafo.
      print(f'tamanho: ' + str(t) + ', densidade: ' + str(d))
      print('Carvalho:')
      Carvalho(G,t)
      print('Rodrigues')
      Rodrigues(G,t)


main()