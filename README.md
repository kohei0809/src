# src
実行方法：
    eclipseで実行
    App.javaのメソッドから，実行する実験を選択

eTest:
    実験環境を構築し，イベントを発生させるまで

rTest2:
    環境内をエージェントに巡回させ，イベント処理をさせるまで(手動)

rTest:
    AMTDSやAMTDS/LDまで

cTest:
    エージェント間の交渉を追加し，AMTDS/EDCまで

sTest:
    計画停止の追加まで

pTest:
    計画停止前の重要度の受け渡しを追加し，AMTDS/THEまで

lTest:
    lynnさんのHomingとPausingの追加まで

tTest:
    AMTDS/ER, AMTDS/ERCまで

oTest:
    品質要求を満たしたうえで，1体ずつ不要なエージェントを停止する
    (Deactivation on CountとDeactivation on Time)

mTest:
    品質要求を満たしたうえで，複数体ずつ不要なエージェントを停止する
    (Deactivation on Countに対してしか行っておらず，一気に複数体ずつ停止する必要がないため，このメソッドは不要)

uTest:
    評価指標をD(s)ではなく，U(s)にしたバージョン
    D(s)は平均値が品質要求を超えなければ良かったが，U(s)は最大値が超えないようにするため，未完成

------------------------------------------------------------------------------------------------------------------------

AgentType.hand                      &   BehaviorType.normal             ---->   手動
AgentType.normal                    &   BehaviorType.normal             ---->   AMTDS
AgentType.PDALearning               &   BehaviorType.normal             ---->   AMTDS/LD
AgentType.Communicating             &   BehaviorType.communicable       ---->   AMTDS/EDC
AgentType.PlannedStopping           &   BehaviorType.plannedStoppable   ---->   AMTDS/THE
AgentType.Homing                    &   BehaviorType.normal             ---->   Homingのみ
AgentType.Pausing                   &   BehaviorType.normal             ---->   Pausingのみ
AgentType.HomingAndPausing          &   BehaviorType.normal             ---->   Homingに必ずPausing
AgentType.TimeChange                &   BehaviorType.normal             ---->   AMTDS/ER
AgentType.TimeChange_Communication  &   BehaviorType.communicable       ---->   AMTDS/ERC
AgentType.TimeChange                &   BehaviorType.onebyoneStoppable  ---->   Deactivation on Count
AgentType.TimeChange                &   BehaviorType.clusterStoppable   ---->   Deactivation on Time
AgentType.TimeChange                &   BehaviorType.changeRequirement  ---->   途中で品質要求値を変更

※Kを学習しない場合，初期値の引数を-1にする

------------------------------------------------------------------------------------------------------------------------

p(v)を学習する際には...
ShortestGreedyPathPlanner.java, SubgoalPathPlanner.java, ForMyopiaGreedy.java, GreedyTargetDecider.javaでのexpectationへの代入をコメントアウトが必須!!

------------------------------------------------------------------------------------------------------------------------

GraphShowingにグラフの描画等のpythonプログラムがあるが，画像の保存先ディレクトリに注意
([../result]に保存するのか，外部のHDDに保存するのか)