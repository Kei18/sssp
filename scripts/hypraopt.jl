"""experimental scripts for hyper-parameter optimization"""

using MRMP
import Random: seed!
using Hyperopt
import YAML
import Printf: @sprintf
import Base.Threads
import Dates
import JLD2
import Logging

include("./utils.jl")

function main(args...; kwargs...)
    # load experimental setting
    config = get_config(args...; kwargs...)
    num_search_times = get(config, "num_search_times", 100)
    typeof(num_search_times) != Int && (num_search_times = parse(Int, num_search_times))
    time_limit_sec = get(config, "time_limit_sec", 30)
    typeof(time_limit_sec) != Int && (time_limit_sec = parse(Int, time_limit_sec))

    # prepare directory
    date_str = replace(string(Dates.now()), ":" => "-")
    root_dir = joinpath(pwd(), "..", "data", "hypra", date_str)
    !isdir(root_dir) && mkpath(root_dir)

    # save configuration file
    save_config(config, root_dir, date_str)

    # load benchmark
    I = JLD2.load(config["benchmark_file"], "instances")

    # pre-compile
    args = get_solver_args(first(I))
    Threads.@threads for solver_info in config["solvers"]
        eval(Meta.parse(solver_info["target"]))(args...; TIME_LIMIT=time_limit_sec)
    end

    num_solvers = length(config["solvers"])
    num_instances = length(I)
    num_total_tasks = num_solvers * num_instances * num_search_times

    # optimization
    cnt_fin_all = Threads.Atomic{Int}(0)
    for (k, solver_info) in enumerate(config["solvers"])
        solver_name = solver_info["target"]
        params_cands = Dict()
        foreach(e -> params_cands[Symbol(first(e))] = last(e), solver_info["params"])
        ho = Hyperoptimizer(num_search_times; params_cands...)

        for (i, params...) in ho
            solver =
                (args..., ; kwargs...) ->
                    eval(Meta.parse(solver_name))(args...; params..., kwargs...)
            score = Threads.Atomic{Float64}(0)
            cnt_fin = Threads.Atomic{Int}(0)
            iterators = get_solver_args.(I)
            Threads.@threads for args in iterators
                t = @elapsed begin
                    solution, _ = solver(args...; TIME_LIMIT=time_limit_sec)
                end
                isnothing(solution) && Threads.atomic_add!(score, 1.0 + t * 0.0001)
                Threads.atomic_add!(cnt_fin, 1)
                Threads.atomic_add!(cnt_fin_all, 1)
                print(
                    "\r" * @sprintf(
                        "%6d/%6d (%3d%%)\tsolver:%d/%d %12s\tparams:%4d/%4d\tinstances:%4d/%4d",
                        cnt_fin_all[],
                        num_total_tasks,
                        cnt_fin_all[] / num_total_tasks * 100,
                        k,
                        num_solvers,
                        last(split(solver_name, ".")),
                        i,
                        num_search_times,
                        cnt_fin[],
                        num_instances
                    ),
                )
            end
            push!(ho.results, score[])
        end
        YAML.write_file(
            joinpath(root_dir, "best_params_$(solver_name).yaml"),
            Dict("target" => solver_name, "params" => Dict(zip(ho.params, ho.minimizer))),
        )
        println("\n", ho)
    end
    postprocessing(config)
end
