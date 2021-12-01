@testset "point2d" begin
    include("../example/plan_point2d.jl")
    @test S != nothing
end

@testset "point3d" begin
    include("../example/plan_point3d.jl")
    @test S != nothing
end

@testset "line2d" begin
    include("../example/plan_line2d.jl")
    @test S != nothing
end

@testset "arm2" begin
    include("../example/plan_arm2.jl")
    @test S != nothing
end

@testset "car" begin
    include("../example/plan_car.jl")
    @test S != nothing
end
