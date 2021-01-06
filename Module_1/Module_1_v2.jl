using JuMP, LinearAlgebra, DataFrames, Gurobi
using Printf, PrettyTables
# Define input data about the system
# ---------------------------------
# SETS

G = ["G1","G2"] # Generators
N = ["N1","N2","N3"] # Buses
D = ["D1","D2"] # Demands

# ---------------------------------
# DATA

# Network topology
map_N = Dict(
    "N1" => ["N2", "N3"],
    "N2" => ["N1", "N3"],
    "N3" => ["N1", "N2"])


haskey(map_N, "N1")


# Location of generators
map_G = Dict(
            "G1" =>"N1",
            "G2" =>"N2")

# Location of demands
map_D = Dict(
            "D1" =>"N2",
            "D2" =>"N3")

# Capacity of generators [MW]
PGmax = Dict("G1" => 100, "G2" => 80)

# Offer price of generators [$ per MWh]
C = Dict("G1" => 12, "G2" => 20)

# Maximum load of demand [MW]
L = Dict("D1" => 100, "D2" => 50)

# Utility of demands [$ per MWh]
U = Dict("D1" => 40, "D2" => 35)

# Capacity of transmission lines [MW]
Fmax = Dict(
    "N1" => Dict("N1" => 0, "N2" => 100, "N3" => 100),
    "N2" => Dict("N1" => 100, "N2" => 0, "N3" => 100),
    "N3" => Dict("N1" => 100, "N2" => 100, "N3" => 0)
)

# Suseptance of transmission lines [\Ohm^{-1}]
B = Dict(
    "N1" => Dict("N1" => 0, "N2" => 500, "N3" => 500),
    "N2" => Dict("N1" => 500, "N2" => 0, "N3" => 500),
    "N3" => Dict("N1" => 500, "N2" => 500, "N3" => 0)
)


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
   @constraint(model, cons6, theta["N1"] == 0)

   # Balance constraint
   @constraint(model, cons7[n in N], -sum(p_G[g] for g in G if map_G[g] == n)
                                     +sum(p_D[d] for d in D if map_D[d] == n)
                                     +sum(f[n,m] for m in N if m in map_N[n]) == 0)

   # ---------------------------------
   # SOLVING COMMANDS
   optimize!(model)

   # ---------------------------------
   # POSTPROCESSING
   filename = "Results_original.txt"
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
            println(io,n*"-"*m*"     ",round(value.(f[n,m]),digits = 2))
         end
      end
      println(io, "\n")

      println(io, "----------- VOLTAGE ANGLE ------------")
      for n in N
         println(io,"Theta "*n*"     ",round(value.(theta[n]),digits = 2))
      end
      println(io, "\n")

      println(io, "----------- CONSUMPTION ------------")
      println(io,"Consumption level of demand d [MW]")
      for d in D
         println(io,"Con "*d*"     ",round(value.(p_D[d]),digits = 2))
      end
      println(io, "\n")

      println(io, "----------- GENERATION ------------")
      println(io,"Production level of generator g [MW]")
      for g in G
         println(io,"Gen "*g*"     ",round(value.(p_G[g]),digits = 2))
      end
      println(io, "\n")

      println(io, "----------- DUALS OF BALANCE ------------")
      for n in N
         println(io,"Node "*n*"     ",round(-dual(cons7[n]),digits = 2))
      end
      println(io, "\n")
   end
end


solve_model(map_N, map_G, map_D, PGmax, C, L, U, Fmax, B, "Results_original.txt")


Fmax = Dict(
    "N1" => Dict("N1" => 0, "N2" => 100, "N3" => 100),
    "N2" => Dict("N1" => 100, "N2" => 0, "N3" => 100),
    "N3" => Dict("N1" => 100, "N2" => 100, "N3" => 0)
)


# Capacity of transmission lines [MW]
Fmax = [[0   100 40],
         [100 0   100],
         [40 100 0]]

solve_model(map_N, map_G, map_D, PGmax, C, L, U, Fmax, B, "Results_changed.txt")
