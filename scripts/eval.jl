using MRMP
import Random: seed!
import CSV
import YAML
import Printf: @printf, @sprintf
import DataFrames: DataFrame
import Logging
import Dates
import Base.Threads

function run!(
    k::Int64,
    instance::Tuple,
    solvers::Vector{Dict{Any, Any}},
    result::Vector{Any};
    seed::Int64=0)

    config_init, config_goal, obstacles, rads = instance

    # setup search details
    q = config_init[1]
    connect = gen_connect(q, rads, obstacles)
    collide = gen_collide(q, rads)
    check_goal = MRMP.gen_check_goal(config_goal)

    # solve
    for (l, solver_info) in enumerate(solvers)
        solver = Meta.parse(solver_info["_target_"])
        params =  Dict([
            (Symbol(key), val) for (key, val) in filter(e -> e[1] != "_target_", solver_info)
        ])
        seed!(seed)
        t = @elapsed begin
            solution, roadmaps = eval(solver)(
                config_init,
                config_goal,
                connect,
                collide,
                check_goal;
                params...,
            )
        end
        if !MRMP.validate(config_init, connect, collide, check_goal, solution)
            @error @sprintf("%s yields invalid solution for instance-%d, seed=%d",
                            solver_info["_target_"], k, seed)
        end

        result[length(solvers)*(k-1)+l] = (
            instance = k,
            N = length(config_init),
            num_obs = length(obstacles),
            solver = solver_info["_target_"],
            solver_index = l,
            elapsed_sec = t,
            solved = !isnothing(solution),
        )
    end
end

function main(config::Dict; pre_compile::Bool = false)

    # create result directory
    date_str = string(Dates.now())
    root_dir = joinpath(
        get(config, "root", joinpath(@__DIR__, "..", "..", "data", "exp")),
        date_str,
    )
    if !pre_compile
        @info @sprintf("result will be saved in %s", root_dir)
        if !isdir(root_dir)
            mkpath(root_dir)
        end
        additional_info = Dict(
            "git_hash" => read(`git log -1 --pretty=format:"%H"`, String),
            "date" => date_str,
        )
        YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))
    end

    # instance generation
    seed_offset = get(config, "seed_offset", 0)
    num_instances = get(config, "num_instances", 3)
    @info @sprintf("generating %d instances", num_instances)
    instances = begin
        params = Dict([(Symbol(key), val) for (key, val) in config["instance"]])
        delete!(params, Symbol("_target_"))
        target = Meta.parse(config["instance"]["_target_"])
        map(e -> begin
            seed!(e + seed_offset)
            eval(target)(; params...)
        end, 1:num_instances)
    end
    if !pre_compile && get(config, "save_instance_images", false)
        @info "saving instance images"
        for k in 1:num_instances
            plot_instance!(
                instances[k]...;
                filename = joinpath(root_dir, @sprintf("%04d_ins.png", k)),
            )
        end
    end

    num_solvers = length(config["solvers"])
    num_total_tasks = num_instances * num_solvers
    cnt_fin = Threads.Atomic{Int}(0)
    result = Array{Any}(undef, num_total_tasks)
    @info @sprintf("start solving with %d threads", Threads.nthreads())
    Threads.@threads for k = 1:num_instances
        run!(k, instances[k], config["solvers"], result; seed=seed_offset)
        Threads.atomic_add!(cnt_fin, num_solvers)
        @printf("\r%04d/%04d tasks have been finished", cnt_fin[], num_total_tasks)
    end
    println()

    # save result
    if !pre_compile
        @info("save result")
        CSV.write(joinpath(root_dir, "result.csv"), result)
    end
end

# read experimental setting
function main(args::Vector{String})
    config_file = args[1]
    if !isfile(config_file)
        @warn @sprintf("%s does not exists", config_file)
        return
    end
    config = YAML.load_file(config_file)

    # parse arguments
    for k = 2:length(args)
        keys, val = split(args[k], "=")
        _config = config
        keys_arr = split(keys, ".")
        for key in keys_arr[1:end-1]
            if haskey(_config, key)
                _config = _config[key]
            else
                @error @sprintf(
                    "%s does not have key %s",
                    config_file,
                    join(keys_arr[1:end-1], ".")
                )
                return
            end
        end

        # parse type
        val_parsed = tryparse(Int64, val)
        if isnothing(val_parsed)
            val_parsed = tryparse(Float64, val)
        end
        if isnothing(val_parsed)
            val_parsed = val
        end

        _config[keys_arr[end]] = val_parsed
    end

    # run once to force compilation
    @info "pre-compilation"
    Logging.with_logger(Logging.SimpleLogger(stdout, Logging.Error)) do
        main(merge(config, Dict("num_instances" => 1)); pre_compile = true)
    end

    # start experiment
    @info "done, start performance measurement"
    main(config)
end

@time main(ARGS)
