export StatePoint2D, StatePoint3D

abstract type AbsState end
abstract type StatePoint <: AbsState end

struct StatePoint2D <: StatePoint
    x::Float64
    y::Float64
end

struct StatePoint3D <: StatePoint
    x::Float64
    y::Float64
    z::Float64
end
