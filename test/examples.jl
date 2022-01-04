@testset "point2d" begin
    include("../example/plan_point2d.jl")
end

@testset "point3d" begin
    include("../example/plan_point3d.jl")
end

@testset "line2d" begin
    include("../example/plan_line2d.jl")
end

@testset "arm2" begin
    include("../example/plan_arm2.jl")
end

@testset "car" begin
    include("../example/plan_car.jl")
end

@testset "refinment" begin
    include("../example/refine_point2d.jl")
end

@testset "tpg" begin
    include("../example/smoothing_point2d.jl")
end
