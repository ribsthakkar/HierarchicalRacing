import subprocess


def run_win_cmd(cmd):
    d = {}
    d["PATH"] = "C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib;"
    retcode = subprocess.call(cmd, env=d)
    if retcode != 0:
        raise Exception('cmd %s failed, see above for details', cmd)

print("LONG STRAIGHT SCENARIO 1")
results = {}
for tg in range(5, 10):
    for p1v in range(2, 5):
        for p0v in range(2, 5):
            if (tg, p1v, p0v) in results: continue
            mt = 43 + tg

            command = [f"C:/JDK/OpenJDK/jdk-11/bin/java.exe",
                       f"-Xmx128g",
                       f"-Xss1g",
                       f"-Djava.library.path=C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib",
                       f"-javaagent:C:/Program Files/JetBrains/IntelliJ IDEA 2020.1.2/lib/idea_rt.jar=54802:C:/Program Files/JetBrains/IntelliJ IDEA 2020.1.2/bin",
                       f"-Dfile.encoding=UTF-8",
                       f"-classpath",
                       f"C:/Users/Rishabh Thakkar/Documents/prism-games/out/production/prism;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/colt.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/pepa.zip;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jhoafparser.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/epsgraphics.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/log4j.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jas.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jfreechart.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/nailgun-server.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jcommon.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/commons-math3-3.3.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/lp_solve_5.5_java/lib/lpsolve55j.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/lp_solve_5.5_java/lib/junit.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/ppl_java/ppl_java.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/z3/com.microsoft.z3.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/out/production/ext",
                       f"prism.PrismCL",
                       f"../two_player_smg1.prism",
                       f"../two_player_smg.props",
                       f"-prop",
                       f"1",
                       f"-const",
                       f"max_time={mt},num_laps=1,p0_init_ta=0,p1_init_ta=0,p0_init_v={p0v},p1_init_v={p1v},p0_init_tg={tg}"]
            print(command)
            run_win_cmd(command)

print("HAIRPIN SCENARIO 2")
for tg in range(5, 6):
    for p1v in range(2, 5):
        if p1v < 3: continue
        for p0v in range(3, 6):
            if p1v <=3 and p0v < 5: continue
            for p0ta in range(5, 90, 15):
                if p1v <= 3 and p0v <= 5 and p0ta < 50: continue
                for p1ta in range(5, 90, 15):
                    if p1v <= 3 and p0v <= 5 and p0ta <= 50 and p1ta < 20: continue
                    mt = 50+tg
                    command = [f"C:/JDK/OpenJDK/jdk-11/bin/java.exe",
                               f"-Xmx128g",
                               f"-Xss1g",
                               f"-Djava.library.path=C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib",
                               f"-javaagent:C:/Program Files/JetBrains/IntelliJ IDEA 2020.1.2/lib/idea_rt.jar=54802:C:/Program Files/JetBrains/IntelliJ IDEA 2020.1.2/bin",
                               f"-Dfile.encoding=UTF-8",
                               f"-classpath",
                               f"C:/Users/Rishabh Thakkar/Documents/prism-games/out/production/prism;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/colt.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/pepa.zip;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jhoafparser.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/epsgraphics.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/log4j.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jas.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jfreechart.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/nailgun-server.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jcommon.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/commons-math3-3.3.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/lp_solve_5.5_java/lib/lpsolve55j.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/lp_solve_5.5_java/lib/junit.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/ppl_java/ppl_java.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/z3/com.microsoft.z3.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/out/production/ext",
                               f"prism.PrismCL",
                               f"../two_player_smg2.prism",
                               f"../two_player_smg.props",
                               f"-prop",
                               f"1",
                               f"-const",
                               f"max_time={mt},num_laps=1,p0_init_ta={p0ta},p1_init_ta={p1ta},p0_init_v={p0v},p1_init_v={p1v},p0_init_tg={tg}"]
                    print(command)
                    # count+=1
                    run_win_cmd(command)

print("CHICANE SCENARIO 3")
for tg in range(5, 6):
    for p1v in range(2, 5):
        for p0v in range(3, 6):
            for p0ta in range(5, 90, 15):
                for p1ta in range(5, 90, 15):
                    mt = 50+tg
                    command = [f"C:/JDK/OpenJDK/jdk-11/bin/java.exe",
                               f"-Xmx128g",
                               f"-Xss1g",
                               f"-Djava.library.path=C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib",
                               f"-javaagent:C:/Program Files/JetBrains/IntelliJ IDEA 2020.1.2/lib/idea_rt.jar=54802:C:/Program Files/JetBrains/IntelliJ IDEA 2020.1.2/bin",
                               f"-Dfile.encoding=UTF-8",
                               f"-classpath",
                               f"C:/Users/Rishabh Thakkar/Documents/prism-games/out/production/prism;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/colt.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/pepa.zip;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jhoafparser.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/epsgraphics.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/log4j.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jas.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jfreechart.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/nailgun-server.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/jcommon.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/lib/commons-math3-3.3.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/lp_solve_5.5_java/lib/lpsolve55j.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/lp_solve_5.5_java/lib/junit.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/ppl_java/ppl_java.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/prism/ext/z3/com.microsoft.z3.jar;C:/Users/Rishabh Thakkar/Documents/prism-games/out/production/ext",
                               f"prism.PrismCL",
                               f"../two_player_smg3.prism",
                               f"../two_player_smg.props",
                               f"-prop",
                               f"1",
                               f"-const",
                               f"max_time={mt},num_laps=1,p0_init_ta={p0ta},p1_init_ta={p1ta},p0_init_v={p0v},p1_init_v={p1v},p0_init_tg={tg}"]
                    print(command)
                    # count+=1
                    run_win_cmd(command)
