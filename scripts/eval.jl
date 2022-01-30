"""experimental scripts to evaluate solvers"""

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
    solvers::Vector{Dict{Any,Any}},
    result::Vector{Any};
    time_limit::Real = 10,
    seed::Int64 = 0,
    root_dir::String = "",
    save_animation::Bool = false,
    anim_plot_params::Dict = Dict(),
)::Nothing

    config_init, config_goal, obstacles, ins_params... = instance

    # setup search details
    q = config_init[1]
    connect = gen_connect(q, obstacles, ins_params...)
    collide = gen_collide(q, ins_params...)
    check_goal = MRMP.gen_check_goal(config_goal)

    # solve
    for (l, solver_info) in enumerate(solvers)
        solver_name = solver_info["_target_"]
        solver = Meta.parse(solver_name)
        params = Dict([
            (Symbol(key), val) for
            (key, val) in filter(e -> e[1] != "_target_", solver_info)
        ])
        seed!(seed)
        t_planning = @elapsed begin
            solution, roadmaps = eval(solver)(
                config_init,
                config_goal,
                connect,
                collide,
                check_goal;
                TIME_LIMIT = time_limit,
                params...,
            )
        end
        if !MRMP.validate(config_init, connect, collide, check_goal, solution)
            @error @sprintf(
                "%s yields invalid solution for instance-%d, seed=%d",
                solver_name,
                k,
                seed
            )
        end
        # compute solution quality
        cost_original = get_solution_cost(solution)
        t_refinement = @elapsed begin
            res_refined = smoothing(solution, connect, collide)
        end

        result[length(solvers)*(k-1)+l] = (
            instance = k,
            N = length(config_init),
            num_obs = length(obstacles),
            solver = solver_name,
            solver_index = l,
            solved = !isnothing(solution),
            elapsed_planning = t_planning,
            elapsed_refinement = t_refinement,
            elapsed_total = t_planning + t_refinement,
            sum_of_cost_original = isnothing(cost_original) ? 0 :
                                   cost_original[:sum_of_cost],
            makespan_original = isnothing(cost_original) ? 0 : cost_original[:makespan],
            sum_of_cost_refined = isnothing(res_refined) ? 0 :
                                  res_refined[end][:sum_of_cost],
            makespan_refined = isnothing(res_refined) ? 0 : res_refined[end][:makespan],
        )

        save_animation &&
            Threads.nthreads() == 1 &&
            !isnothing(solution) &&
            plot_anim!(
                config_init,
                config_goal,
                obstacles,
                ins_params...;
                solution = solution,
                filename = "$(root_dir)/res_$(solver_name)_$(k).gif",
                anim_plot_params...,
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
        seed!(seed_offset)
        map(e -> eval(target)(; params...), 1:num_instances)
    end
    if !pre_compile && Bool(get(config, "save_instance_images", false))
        @info "saving instance images"
        for k = 1:num_instances
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
    time_limit = get(config, "time_limit", 10)
    save_animation = !pre_compile && Bool(get(config, "save_animation", false))
    anim_plot_params =
        Dict(Symbol(key) => val for (key, val) in get(config, "anim_plot_params", Dict()))

    @info @sprintf("start solving with %d threads", Threads.nthreads())
    Threads.@threads for k = 1:num_instances
        run!(
            k,
            instances[k],
            config["solvers"],
            result;
            time_limit = time_limit,
            seed = seed_offset,
            root_dir = root_dir,
            save_animation = save_animation,
            anim_plot_params = anim_plot_params,
        )
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
function main(args...)
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
        main(
            merge(config, Dict("num_instances" => 1, "time_limit" => 10));
            pre_compile = true,
        )
    end

    # start experiment
    @info "done, start performance measurement"
    main(config)
end

main() = main(ARGS...)
