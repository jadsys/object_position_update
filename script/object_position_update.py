#!/usr/bin/env python3


import rospy
import ruamel
import numpy as np
import quaternion
import copy
import os

from os.path import expanduser
from ruamel.yaml import YAML, add_constructor, resolver
from collections import OrderedDict
from datetime import datetime, timedelta, timezone
from uoa_poc6_msgs.msg import r_objects_location
from uoa_poc6_msgs.msg import r_get_objects_location

# 入力時に順序を保持する
add_constructor(resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    lambda loader, node: OrderedDict(loader.construct_pairs(node)))

yaml = ruamel.yaml.YAML()
yaml.default_flow_style = False

def readParam():
    """ノードのパラメータ設定

    パラメータの読み込みを行う

    Args:
        None

    Returns:
        None

    Raises:
        None

    Yields:
        None

    Examples:

        >>> readParam()

    Note:
        読み込めなかった場合はデフォルト値が入る

    """
    global cnoid_project_file_dir
    global input_cnoid_project_file_name
    global output_cnoid_project_file_name
    global sub_obj_loc_topic_name
    global pub_get_obj_loc_topic_name
    global update_list_table
    global is_overwrite
    
    # パラメータの取得
    home_dir = expanduser("~")
    params = rospy.get_param("~", {}) # paramsはdict型
    # プロジェクトファイルの保存先
    cnoid_project_file_dir = params.get("cnoid_project_file_dir", home_dir + "/catkin_ws/src/object_position_update/test")
    # 読み込むプロジェクトファイル名
    input_cnoid_project_file_name = params.get("input_cnoid_project_file_name", "test_input.cnoid")
    # 書き込むプロジェクトファイル名
    output_cnoid_project_file_name = params.get("output_cnoid_project_file_name", "test_output.cnoid")
    # オーバーライトするか
    is_overwrite = params.get("output_file_overwrite", False)
    # オブジェクトの位置受信トピック名
    sub_obj_loc_topic_name = params.get("sub_obj_loc_topic_name", "/simulator_bridge/object_location")
    # オブジェクトの位置取得配信トピック名
    pub_get_obj_loc_topic_name = params.get("pub_get_obj_loc_topic_name", "/simulator_bridge/get_object_location")
    # 更新対象の準静的物体テーブル
    update_list_table = params.get("update_item_list", [{'body_file': '${SHARE}/LICTiA/model/Table-120x120.body', 'name': 'Table', 'offset_z': 0.72, 'top_layer': 'FreeSpace', 'update_object': 'Table-120x120'}, {'body_file': '${SHARE}/LICTiA/model/Office_chair.body', 'name': 'Chair', 'offset_z': 0.45, 'top_layer': 'FreeSpace', 'update_object': 'Office_chair'}])
    
def readProjectFile():
    """Choreonoidプロジェクトファイル読み込み関数

    Choreonoidのプロジェクトを読み込む。

    Args:
        None

    Returns:
        bool: 読み込み成功の可否。Trueは読み込み成功、Falseは失敗。

    Raises:
        None

    Yields:
        None

    Examples:

        >>> readProjectFile()

    Note:
        読み込む.cnoidファイルがない場合などのエラーの場合は、
        読み込みせずにエラーメッセージを表示する。

    """
    rospy.loginfo("---- Read project files ----")
    global dict_prj
    
    result = False

    filepath = cnoid_project_file_dir + '/' + input_cnoid_project_file_name
    rospy.logdebug(filepath)
    
    # Read yaml file.
    try:
        with open(filepath, 'r') as stream:
            rospy.loginfo("File was opened. Loading ...")
            try:
                dict_prj = yaml.load(stream) # dict_prjは辞書型（class 'ruamel.yaml.comments.CommentedMap'）
            except ruamel.yaml.YAMLError as e:
                rospy.logerr(e)
                rospy.logerr("%s is not YAML", filepath)
            else:
                result = True
                rospy.loginfo("Complete!")
    except FileNotFoundError:
        rospy.logerr('Specified file is not found. The file path is \"%s\".', filepath)
    
    return result    

def writeProjectFile():
    """Choreonoidプロジェクトファイル保存関数

    受信データで書き換えたyamlデータを.cnoidファイルへ出力する

    Args:
        None

    Returns:
        None

    Raises:
        None

    Yields:
        None

    Examples:

        >>> writeProjectFile()

    Note:
        書き込むファイルが既に存在しているかつ上書き設定がFalseの場合に、
        ファイル名が☓☓☓.cnoidから、☓☓☓_yyyymmddhhmmss.cnoidに変更される。
        エラーの場合は書き込みを行わない。

    """
    rospy.loginfo("---- Write project files ----")
    filepath = cnoid_project_file_dir + '/' + output_cnoid_project_file_name
    
    # 上書き確認
    try:
        if not is_overwrite and os.path.exists(filepath): # 既にファイルが存在する場合
            # 上書きしない場合、ファイル名の変更
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            base, ext = os.path.splitext(filepath)
            new_filename = f"{base}_{timestamp}{ext}"
            os.rename(filepath, new_filename)
        
        # 書き出し
        with open(filepath, 'w') as stream:
            rospy.loginfo("File was opened. Writing ...")
            try:
                yaml.dump(dict_prj, stream)
            except ruamel.yaml.YAMLError as e:
                rospy.logerr(e)
                rospy.logerr(" Writing to the file is skipped.")
            rospy.loginfo("Complete!")
    except Exception as e:
        rospy.logerr('Specified file is not found. The file path is \"%s\". Writing to the file is skipped.', filepath)
            
def analyzeProjectFile():
    """Choreonoidプロジェクトファイル解析関数

    Choreonoidのプロジェクトの解析を行い、要素ごとに分離して保持する。

    Args:
        None

    Returns:
        None

    Raises:
        None

    Yields:
        None

    Examples:

        >>> analyzeProjectFile()

    Note:
        この関数は、'items'、'views'、'toolbars'、'Body'、'viewAreas'、'layoutOfToolBars'  
        のような要素を編集しやすいように辞書から分離します。

    """
    # 要素の分離
    rospy.logdebug("--- AnalyzeProjectFile ---")
    global dict_item
    global dict_root_items
    global dict_views
    global dict_toolbars
    global dict_Body
    global dict_viewAreas
    global dict_layout_of_tool_bars

    # 編集しやすいように各セクション毎に分離
    dict_item = dict_prj.pop('items') # dict_itemは辞書型dict_item<class 'ruamel.yaml.comments.CommentedMap'>
    dict_root_items = dict_item.pop('children', None)
    dict_views = dict_prj.pop('views', None)
    dict_toolbars = dict_prj.pop('toolbars', None)
    dict_Body = dict_prj.pop('Body', None)
    dict_viewAreas = dict_prj.pop('viewAreas', None)
    dict_layout_of_tool_bars = dict_prj.pop('layoutOfToolBars', None)

def update_dict(target, key, value):
    """yamlデータのキー指定での更新

    YAMLのtargetのkeyをvalueで更新する。
    更新成功すればTrue、失敗すればFalseが帰る。
    Args:
        target (list or dict): 検索・更新する対象のリストまたは辞書
        key (str): 'name' フィールドでマッチするキー
        value: 一致した場合に 'children' リストに追加する値

    Returns:
        bool: 更新成功の可否。Trueは成功、Falseはkeyが見つからず失敗

    Raises:
        None

    Yields:
        None

    Examples:
        入れ子になった辞書 'target' を考える:
        
        >>> target = {
                        'name': 'A',
                        'children': [
                            {'name': 'B', 'children': [{'name': 'C'}]},
                            {'name': 'D', 'children': [{'name': 'E'}]}
                        ]
                    }
                
        'name'が'D'に等しい場合に’children’へ、新しい辞書{'name': 'F'}を追加する:
        
        >>> success = update_dict(target, 'D', {'name': 'F'})

    Note:
        この関数は、指定したキーと同じ 'name' キーを持つ辞書を再帰的に検索する。
        また、'name' キーと、入れ子になった辞書のリストを指す 'children' キーを持つ辞書で動作するように設計されている。

    """
    result = False
    if isinstance(target,list): # listの場合
        # list型の要素1つ1つを再サーチ
        for target_item in target:
            result = update_dict(target_item, key, value)
            if result: # 戻り値がnoneじゃない場合は検索完了
                break
    elif isinstance(target, dict):
        if 'name' in target and target['name'] == key:
            # keyと一致したデータがある場合
            res = target['children']
            target['children'].append(value)
            result = True
        else:
            # keyと一致したデータが無い場合
            # 1つ下の階層を再度サーチ
            try:
                result = update_dict(target['children'], key, value)
            except KeyError:
                result = False
    return result

def remove_elements(data_list, target_key, target_val, match_mode = True):
    """要素の削除関数

    アイテムリストのtarget_keyにtarget_valが存在する場合は削除する。

    Args:
        data_list (list): 対象の辞書型のリスト
        target_key (str): 対象のキー
        target_val (str): 対象の値
        value (bool, optional): 検索条件の設定。
            - True（デフォルト）の場合、完全一致
            - Falseの場合、部分一致

    Returns:
        list: 処理済みのデータリスト

    Raises:
        None

    Yields:
        None

    Examples:
        辞書のリスト 'data_list' を例とする:
        
        >>> data_list = [
                            {'name': 'A', 'value': 10, 'children': [{'name': 'B', 'value': 20}]},
                            {'name': 'C', 'value': 30},
                            {'name': 'D', 'value': 40, 'children': [{'name': 'E', 'value': 50}]},
                        ]
        
        'name'が'B'である要素を削除するには、次のようにする:
        
        >>> updated_list = remove_elements(data_list, 'name', 'B')
        
    Note:
        この関数は 'children' キーを持つ辞書を再帰的に検索する。

    """
    temp_list = data_list.copy()  # コピーを作成
    for item in temp_list:  # 要素の取り出し
        if 'children' in item:  # 取り出した要素にchildrenが含まれている場合は再帰呼び出し
            item['children'] = remove_elements(item['children'], target_key, target_val, match_mode)
        if (target_key in item and target_val in  item[target_key] ):
            # match_mode and item[target_key] == target_val or        # 完全一致モード
            # not match_mode and item[target_key] in target_val ):    # 一部を含む場合
            data_list.remove(item)  # 削除
            
    return data_list


def analize_config(data, target_name, serch_key):
    """コンフィグ値の解析関数

    ネストしたコンフィグ値から要素名target_nameで検索し、存在すればserch_keyの要素を取り出す。

    Args:
        data (list): 検索対象のコンフィグ値
        target_name (str): 対象のアイテム名
        serch_key (str): 要素を取り出す対象のキー

    Returns:
        obj: 要素のオブジェクト

    Raises:
        None

    Yields:
        None

    Examples:        
        辞書のリスト 'data' を例にする:
        
        >>> data =  [
                        {'name': 'item1', 'value': 10, 'type': 'A'},
                        {'name': 'item2', 'value': 20, 'type': 'B'},
                        {'name': 'item3', 'value': 30, 'type': 'A'},
                    ]
        
        
        'name'が'item2'に等しい項目の'value'を取得するには、次のように呼び出す:
        
        >>> result = analize_config(data, 'item2', 'value')
        >>> print(result)
        20

    Note:
        特になし

    """
    for item in data:
        if item['name'] == target_name:
            return item.get(serch_key, None)
    return None

# サブスクライバ
def recvObjectPosition(msg):
    """サブスクライバのコールバック関数

    取得した書き換え対象の準静的物体の情報でプロジェクトファイルのデータを書き換える

    Args:
        msg (obj): 受信した準静的物体の情報

    Returns:
        None

    Raises:
        None

    Yields:
        None

    Examples:

        >>> recvObjectPosition(msg)

    Note:
        物体の位置データを受信し、各処理を行い、プロジェクトファイルの項目を更新する。
        受信したデータと更新処理に関する情報を保存する。

    """
    
    rospy.loginfo("Subscribed new object location data.")
    
    global recv_items   # 受信データの格納用
    recv_items = {}
     
    previous_obj_name = ""  # オブジェクト名格納用
    
    for idx, new_object in enumerate(msg.objects, start=1):
        # 同一オブジェクトの場合は１回のみ実行
        if(new_object.name != previous_obj_name ):
            # 変化更新対象のパラメータ取得
            file_dir    = analize_config(update_list_table, new_object.name, "body_file")   # .bodyファイルの読み込み先ディレクトリ
            update_key  = analize_config(update_list_table, new_object.name, "top_layer")    # Choreonoidのアイテムツリービュー上の一つ上の階層のアイテム名
            object_name = analize_config(update_list_table, new_object.name, "update_object") # 更新対象のChoreonoidのアイテムツリービュー上のオブジェクト名
            offset_z    = analize_config(update_list_table, new_object.name, "offset_z")    # choreonoid上のモデルの高さ方向のオフセット値
            
            if file_dir is None: # bodyファイルのディレクトリが読めない場合
                rospy.logwarn("The item '%s' is not defined in the update list parameters (update_item_list). It will be ignored.",  new_object.name)
                continue
            
            # 更新前に対象のオブジェクトを削除
            remove_elements(dict_root_items, 'name', object_name, False)
            
            # アイテム番号のリセット
            item_no = 0
        
        position = [new_object.new_pose.position.x, new_object.new_pose.position.y, offset_z] # dict
        # アイテムリストの作成    
        new_item = {
            'id': idx,
            'name': f"{object_name}-{item_no}",
            'plugin': 'Body',
            'class': 'BodyItem',
            'is_checked': True,
            'data': {
                'file': file_dir,
                'format': 'CHOREONOID-BODY',
                'rootPosition': position,
                'rootAttitude': quaternion.as_rotation_matrix(np.quaternion(
                    new_object.new_pose.orientation.w,
                    new_object.new_pose.orientation.x,
                    new_object.new_pose.orientation.y,
                    new_object.new_pose.orientation.z
                )).ravel().tolist(), # クォータニオン⇒回転行列
                'initialPosition': position,
                'initialAttitude': quaternion.as_rotation_matrix(np.quaternion(
                    new_object.new_pose.orientation.w,
                    new_object.new_pose.orientation.x,
                    new_object.new_pose.orientation.y,
                    new_object.new_pose.orientation.z
                )).ravel().tolist(), # クォータニオン⇒回転行列
                'fix_root': True,
                'collisionDetection': True,
                'selfCollisionDetection': False,
                'lock_location': False,
                'scene_sensitive': True,
                'zmp': [0, 0, 0]
            }
        }
        
        previous_obj_name = new_object.name
        item_no += 1 

        rospy.logdebug("------------------ New item ------------------")
        rospy.logdebug("{new_object}")
        

        rospy.logdebug("------------------ Before ------------------")
        rospy.logdebug("{dict_root_items}")
        
        update_dict(dict_root_items, update_key, new_item)

        rospy.logdebug("------------------ Updated ------------------")
        rospy.logdebug("{dict_root_items}")
        
    # projectファイル出力
    # 結合
    dict_prj.update(items=dict_item)
    dict_prj['items'].update(children=dict_root_items)
    dict_prj.update(views=dict_views)
    dict_prj.update(toolbars=dict_toolbars)
    dict_prj.update(Body=dict_Body)
    dict_prj.update(viewAreas=dict_viewAreas)
    dict_prj.update(layoutOfToolBars=dict_layout_of_tool_bars)
    
    rospy.logdebug("------------------ writeProjectFile ------------------")
    rospy.loginfo("{dict_prj}")
    
    # 書き込み
    writeProjectFile()

if __name__ == '__main__':
    # 初期化宣言
    rospy.init_node('object_position_update', anonymous=True)
    
    # ROSパラメータ読み込み
    readParam()

    # プロジェクトファイル読み込み
    if readProjectFile():
        
        # プロジェクトファイルの解析
        analyzeProjectFile()
        
        # サブスクライバ定義
        rospy.Subscriber(sub_obj_loc_topic_name, r_objects_location, recvObjectPosition)
        
        rospy.sleep(1)  
        
        # 受信待機
        rospy.spin()
    
    else:
        rospy.logerr("Abort")