export StatePoint2D, StatePoint3D, StateLine2D, StateArm2

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

struct StateLine2D <: AbsState
    x::Float64
    y::Float64
    theta::Float64
end

struct StateArm2 <: AbsState
    x::Float64
    y::Float64
    theta1::Float64
    theta2::Float64
end
