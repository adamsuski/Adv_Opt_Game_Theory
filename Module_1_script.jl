using JuMP, LinearAlgebra, DataFrames, Gurobi
using Printf, PrettyTables
# Define input data about the system
# ---------------------------------
# SETS

#G = ["G1","G2"] # Generators
G = collect(1:2)
#N = ["N1","N2","N3"] # Buses
N = collect(1:3)
#D = ["D1","D2"] # Demands
D = collect(1:2)

# ---------------------------------
# DATA

# Network topology
map_N = [[0 1 1],
         [1 0 1],
         [1 1 0]]

# Location of generators
map_G = [[1 0 0],
         [0 1 0]]

# Location of demands
map_D = [[0 1 0],
         [0 0 1]]

# Capacity of generators [MW]
PGmax = [100, 80];

# Offer price of generators [$ per MWh]
C = [12, 20];

# Maximum load of demand [MW]
L = [100, 50];

# Utility of demands [$ per MWh]
U = [40, 35];

# Capacity of transmission lines [MW]
Fmax = [[0   100 100],
         [100 0   100],
         [100 100 0]]

# Suseptance of transmission lines [Ohm^{-1}]
B =     [[0   500 500],
          [500 0   500],
          [500 500 0]]


# In this cell I create function solve_ed, which solves the  problem for a given set input parameters.
function solve_model(map_N, map_G, map_D, PGmax, C, L, U, Fmax, B, filename)

   # Declare Gurobi model
   model = Model(Gurobi.Optimizer)

   # ---------------------------------
   # VARIABLES

   # Social welfare of the market [$]
   SW = @variable(model, SW)

   # Power flow from bus n to bus m [MW]
   f = @variable(model, f[n in N, m in N])

   # Voltage angle of bus n [rad.]
   theta = @variable(model, theta[n in N])

   # Consumption level of demand d [MW]
   p_D = @variable(model, 0<=p_D[d in D])

   # Production level of generator g [MW]
   p_G = @variable(model, 0<=p_G[g in G])

   # ---------------------------------
   # EQUATIONS

   # Obnective function
   @objective(model, Max, SW)

   # Definition of the social welfare
   @constraint(model, cons1, SW == sum(U[d]*p_D[d] for d in D) - sum(C[g]*p_G[g] for g in G))

   # Limit on the generators
   @constraint(model, cons2[g in G], p_G[g] <= PGmax[g])

   # Limit on the loads
   @constraint(model, cons3[d in D], p_D[d] <= L[d])

   # Constraint defining the transmission
   @constraint(model, cons4[n in N, m in N], f[n,m] == B[n][m] * (theta[n] - theta[m]))

   # Limit on the transmission
   @constraint(model, cons5[n in N, m in N], f[n,m] <= Fmax[n][m])

   # Initialization of theta
   @constraint(model, cons6, theta[1] == 0)

   # Balance constraint
   @constraint(model, cons7[n in N], -sum(p_G[g] for g in G if map_G[g][n] != 0)
                                     +sum(p_D[d] for d in D if map_D[d][n] != 0)
                                     +sum(f[n,m] for m in N if map_N[n][m] != 0) == 0)

   # ---------------------------------
   # SOLVING COMMANDS
   optimize!(model)

   # ---------------------------------
   # POSTPROCESSING

   if isfile(filename)
      rm(filename)
   end
   open(filename,"a") do io
      println(io, "----------- OBJ FUNCTION ------------")
      println(io,"Social welfare of the market [\$]")
      println(io,"OBJECTIVE VALUE:"*"     ",round(objective_value(model)),"\n")
      println(io, "----------- TRANSMISSION ------------")
      println(io,"Power flow from bus n to m [MW]")
      for n in N
         for m in N
            println(io,"N"*string(n)*"-"*string(m)*"     ",round(value.(f[n,m]),digits = 2))
         end
      end
      println(io, "\n")

      println(io, "----------- VOLTAGE ANGLE ------------")
      for n in N
         println(io,"Theta "*string(n)*"     ",round(value.(theta[n]),digits = 2))
      end
      println(io, "\n")

      println(io, "----------- CONSUMPTION ------------")
      println(io,"Consumption level of demand d [MW]")
      for d in D
         println(io,"Con "*string(d)*"     ",round(value.(p_D[d]),digits = 2))
      end
      println(io, "\n")

      println(io, "----------- GENERATION ------------")
      println(io,"Production level of generator g [MW]")
      for g in G
         println(io,"Gen "*string(g)*"     ",round(value.(p_G[g]),digits = 2))
      end
      println(io, "\n")

      println(io, "----------- DUALS OF BALANCE ------------")
      for n in N
         println(io,"Node "*string(n)*"     ",round(-dual(cons7[n]),digits = 2))
      end
      println(io, "\n")
   end
end


solve_model(map_N, map_G, map_D, PGmax, C, L, U, Fmax, B, "Results_original.txt")

# Capacity of transmission lines [MW]
Fmax = [[0   100 40],
         [100 0   100],
         [40 100 0]]

solve_model(map_N, map_G, map_D, PGmax, C, L, U, Fmax, B, "Results_changed.txt")
