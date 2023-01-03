"""experimental scripts for hyper-parameter optimization"""

using MRMP
import Random: seed!
using Hyperopt
import YAML
import Printf: @printf, @sprintf
import Base.Threads
import Dates
import JLD2
import Logging

function get_solver_args(ins)
    config_init, config_goal, obstacles, ins_params... = ins
    connect = gen_connect(config_init[1], obstacles, ins_params...)
    collide = gen_collide(config_init[1], ins_params...)
    check_goal = gen_check_goal(config_goal)
    return [config_init, config_goal, connect, collide, check_goal]
end

function main(config_file::String)
    # load experimental setting
    config = YAML.load_file(config_file)
    num_search_times = get(config, "num_search_times", 100)
    time_limit_sec = get(config, "time_limit_sec", 30)

    # prepare directory
    date_str = replace(string(Dates.now()), ":" => "-")
    root_dir = joinpath(pwd(), "..", "data", "hypra", date_str)
    !isdir(root_dir) && mkpath(root_dir)

    # save configuration file
    io = IOBuffer()
    versioninfo(io, verbose=true)
    additional_info = Dict(
        "git_hash" => read(`git log -1 --pretty=format:"%H"`, String),
        "date" => date_str,
        "nthreads" => Threads.nthreads(),
        "env" => String(take!(io)),
    )
    YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))

    # load benchmark
    I = JLD2.load(config["benchmark_file"], "instances")
    num_instances = length(I)

    # pre-compile
    args = get_solver_args(first(I))
    Threads.@threads for solver_info in config["solvers"]
        eval(Meta.parse(solver_info["target"]))(args...; TIME_LIMIT=time_limit_sec)
    end

    # optimization
    results = Dict()
    for solver_info in config["solvers"]
        solver_name = solver_info["target"]
        println("hyper parameter search for $(solver_name) with $(num_search_times) samples " *
            "with $(Threads.nthreads()) threads, timeout: $(time_limit_sec) sec")
        params_cands = Dict()
        foreach(e -> params_cands[Symbol(first(e))] = last(e), solver_info["params"])
        ho = Hyperoptimizer(num_search_times; params_cands...)

        for (i, params...) in ho
            solver = (args..., ; kwargs...) -> eval(Meta.parse(solver_name))(args...; params..., kwargs...)
            score = Threads.Atomic{Float64}(0)
            iterators = get_solver_args.(I)
            Threads.@threads for args in iterators
                t = @elapsed begin
                    solution, _ = solver(args...; TIME_LIMIT = time_limit_sec)
                end
                isnothing(solution) && Threads.atomic_add!(score, 1.0 + t * 0.0001)
            end
            print("\rfin: $(i)/$(num_search_times), failure: $(score[] |> floor |> Int)/$(num_instances)")
            push!(ho.results, score[])
        end
        results[solver_name] = ho
        println("\nsolver:$(solver_name)\n$(ho)")
        YAML.write_file(
            joinpath(root_dir, "best_params_$(solver_name).yaml"),
            Dict(solver_name => Dict(zip(ho.params, ho.minimizer))),
        )
    end
end
