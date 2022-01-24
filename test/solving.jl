using MRMP
import Random: seed!

solvers = [
    "MRMP.PP",
    "MRMP.CBS",
    "MRMP.RRT",
    "MRMP.RRT_connect",
    "MRMP.PRM_direct",
    "MRMP.planner1",
]

TIME_LIMIT = 1.0

ins_commons_params = Dict(:N => 3, :rad => 0.025, :num_obs => 3, :rad_obs => 0.05)

function test_all_solvers(ins_gen_fn)
    seed!(1)
    config_init, config_goal, obstacles, ins_params... = ins_gen_fn(; ins_commons_params...)
    connect = gen_connect(config_init[1], obstacles, ins_params...)
    collide = gen_collide(config_init[1], ins_params...)
    check_goal = MRMP.gen_check_goal(config_goal)

    for solver in solvers
        @info solver
        seed!(1)
        @time eval(Meta.parse(solver))(
            config_init,
            config_goal,
            connect,
            collide,
            check_goal;
            TIME_LIMIT = TIME_LIMIT,
            VERBOSE = 1,
        )
    end
end

@testset "point2d" begin
    test_all_solvers(MRMP.gen_random_instance_StatePoint2D)
end
@testset "point3d" begin
    test_all_solvers(MRMP.gen_random_instance_StatePoint3D)
end
@testset "line2d" begin
    test_all_solvers(MRMP.gen_random_instance_StateLine2D)
end
@testset "arm22" begin
    test_all_solvers(MRMP.gen_random_instance_StateArm22)
end
@testset "arm33" begin
    test_all_solvers(MRMP.gen_random_instance_StateArm33)
end
@testset "snake2d" begin
    test_all_solvers(MRMP.gen_random_instance_StateSnake2D)
end
@testset "dubins2d" begin
    test_all_solvers(MRMP.gen_random_instance_StateDubins)
end
