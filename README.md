# slam_gmapping_fix_edit
a copy of slam_gmapping. add some new abilities.

改造したslam_gmapping_fix PKGは以下の機能を追加しました。

１．始点位置指定
２．地図指定（１．始点指定の必要がある。なけらば、変な事が起こす可能性がある）
３．第三位置入力によるパーティクルのEKF位置更新と重み更新

１と２はParameterにより、Node起動前に指定する事が可能。もしくは、指定のTopicで起動後指定や変換する。


以下のParameterを追加
１）地図指定
map_yaml_file_name
map_pgm_file_name

２）始点位置指定
initial_pose_x
initial_pose_y
initial_pose_a

３）始点位置の分散範囲指定
initial_cov_xx
initial_cov_yy
initial_cov_aa


以下のTopic（Msg）を追加
１）gmapping_init_map(nav_msgs::OccupancyGrid)
現在の位置上に、地図と位置の指定値を書き換える。
位置の指定値が取る前にSLAMに反映出来ません。
＊略に使えない


２）gmapping_init_pose(geometry_msgs::Pose)
現在の地図上に、新たな自己位置に変動する。
＊１,指定した地図ある場合、SLAMを再起動＊する。
＊２,指定した地図無い場合、位置の指定値を書き換える、パーティクルの位置を強制出来にこの指定値になる。

３）gmapping_init_mapPose(gmapping_fix::gmapping_initMapPoseMsg)
地図と位置の指定値を書き換えて、SLAMを再起動する。
今まで作った地図がなくなる。

４）gmapping_init_poseCov(geometry_msgs::PoseWithCovarianceStamped)
現在の地図上に、位置の指定値を書き換えて、SLAMを再起動＊する。
CovarianceはSLAMを再起動＊の時、分散範囲を指定。
Covarianceは０の場合、Node起動前のParameterを利用する。

５）gmapping_ekf_poseCov(geometry_msgs::PoseWithCovarianceStamped)
第三位置入力によるパーティクルのEKF位置更新と重み更新する。
CovarianceはSLAMを更新する時に使う。
Covarianceは０の場合、Node起動前のParameterを利用する。

SLAMを再起動＊は：
大体のロボット位置を指定して、パーティクルを乱数（範囲Covで指定）に分散する。各パーティクル指定した地図上にICP（地図とレーサーと比べる事）を行う、詳しい位置を取る。詳しい位置でSLAM再開するに表す事。




使う方法：
slam_gmapping_fixとopenslam_gmapping_fixをCatkin_wsにコーピーしてをCatkin_Makeする
slam_gmapping_fix/gmapping_fix/launch/athena_2dslam_fix.launchを起動する。（３６０度レーサーVLP-16が必要）
