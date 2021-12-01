@testset "point2d" begin
    include("../example/point2d.jl")
    @test S != nothing
end

@testset "point3d" begin
    include("../example/point3d.jl")
    @test S != nothing
end

@testset "line2d" begin
    include("../example/line2d.jl")
    @test S != nothing
end

@testset "arm2" begin
    include("../example/arm2.jl")
    @test S != nothing
end

@testset "car" begin
    include("../example/car.jl")
    @test S != nothing
end
