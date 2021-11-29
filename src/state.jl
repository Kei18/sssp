export State, States, StatePoint2D

abstract type AbsState end

struct StatePoint2D <: AbsState
    x::Float64
    y::Float64
end
