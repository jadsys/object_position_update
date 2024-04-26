object_position_update
=======

概要
=======
このパッケージはChoreonoidのプロジェクトファイル（.cnoid）上のオブジェクトの位置と姿勢の書き換えを行います。現実空間のオブジェクトの位置情報を仮想空間に反映することを目的に開発されました。
天井に設置された監視カメラ等から物体の位置の更新を検出し、その位置情報をもとにシミュレータ環境も更新する場合に使用可能です。
なお、入力する位置情報はカメラ座標系からシミュレータ座標系に変換する必要がございます。
また、更新したプロジェクトファイルを毎回再度読み込む必要があるため、リアルタイム性が要求されるシステムでは利用が困難です。

インストール方法
=======
### 1．ROSワークスペースのディレクトリに移動し、リポジトリをクローン
```bash 
cd ~/{ROSワークスペースディレクトリ}/src/
git clone -b "2023年度成果物" https://github.com/jadsys/object_position_update.git
```
### 2．Buildを行う
```bash 
cd object_position_update
catkin build --this
```
### X. 依存関係の解決
当パッケージでは外部パッケージとして以下を利用しております。
- [ruamel.yaml](https://sourceforge.net/projects/ruamel-yaml/)
- [uoa_poc6_msgs](https://github.com/jadsys/uoa_poc6_msgs.git)


その他のパッケージは以下の方法でインストール可能です。
```bash
# vcsツールのインストール（既にインストール済みの場合スキップ）
sudo pip install -U vcstool

# rosdepのインストール
sudo apt install python3-rosdep
sudo rosdep init # 過去に実行済みの場合は実行不要
rosdep update # 過去に実行済みの場合は実行不要

# 依存関係のインストール
cd ~/{ROSワークスペースディレクトリ}/src
vcs import  < object_position_update/dependency.rosinstall
rosdep install -i --from-paths object_position_update
catkin build
```

ライセンス
=======
## BSD 3-Clause License

Copyright (c) 2023, Japan Advanced System,Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software 
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* * *
## 使用ライブラリ関係
### [ruamel.yaml](https://sourceforge.net/projects/ruamel-yaml/)
 [MIT License](https://opensource.org/licenses/MIT):  Copyright (c) 2014-2024 Anthon van der Neut, Ruamel bvba