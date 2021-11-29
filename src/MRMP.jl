module MRMP

include("obstacle.jl")

greet() = println("This is a research repo for multi-robot motion planning. Enjoy planning!")

abstract type State end

export StatePoint2D
struct StatePoint2D <: State
    x::Float64
    y::Float64
end


end
