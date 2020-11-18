
import rclpy
from tmr_msgs.srv import AskItem

import os
import shutil
import sys

from tmr_mod_urdf._modify_urdf import *

def _gen_urdf(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('modify_urdf')

    if len(sys.argv) < 3:
        node.get_logger().warn('usage: modify_urdf_node model new_model replace')
        return

    model = sys.argv[1]
    new_model = sys.argv[2]
    replace = False
    if len(sys.argv) == 4:
        if sys.argv[3].upper() == 'REPLACE':
            replace = True
            node.get_logger().warn('origin urdf file will be replaced')


    ask_item = node.create_client(AskItem, 'tmr/ask_item')
    if not ask_item.wait_for_service(3.0):
        node.get_logger().error('No AskItem service')
        return

    req = AskItem.Request()
    req.wait_time = 1.0

    req.id = 'dh'
    req.item = 'DHTable'
    future = ask_item.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res_dh = future.result()

    req.id = 'dd'
    req.item = 'DeltaDH'
    future = ask_item.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res_dd = future.result()

    if not res_dh.value.startswith('DHTable={') or not res_dh.value.endswith('}'):
        node.get_logger().error('invalid dh')
        return
    if not res_dd.value.startswith('DeltaDH={') or not res_dd.value.endswith('}'):
        node.get_logger().error('invalid delta_dh')
        return

    node.get_logger().info(res_dh.value)
    node.get_logger().info(res_dd.value)

    dh_strs = res_dh.value[9:-1].split(',')
    dd_strs = res_dd.value[9:-1].split(',')
    '''
    res_dh = 'DHTable={0,-90,0,145.1,0,-277,277,-90,0,429,0,0,-187,187,0,0,411.5,0,0,-162,162,90,90,0,-122.2,0,-187,187,0,90,0,106,0,-187,187,0,0,0,114.4,0,-277,277}'
    res_dd = 'DeltaDH={-0.001059821,0.02508766,0.009534874,0,0.001116668,0.06614932,0.308224,0.0287381,0.06797475,-0.0319523,0.3752921,0.06614756,-0.006998898,0.06792655,-0.06083903,0.02092069,0.02965812,-0.1331249,0.06793034,0.02077797,0.08265772,0.03200645,0.01835932,0.06145732,0.08273286,0.6686108,0.6972408,-0.1793097,-0.0794057,1.425708}'
    node.get_logger().info(res_dh)
    node.get_logger().info(res_dd)
    dh_strs = res_dh[9:-1].split(',')
    dd_strs = res_dd[9:-1].split(',')
    '''

    if len(dh_strs) != 42:
        node.get_logger().error('invalid dh')
        return
    if len(dd_strs) != 30:
        node.get_logger().error('invalid delta_dh')
        return

    dh = [float(i) for i in dh_strs]
    dd = [float(i) for i in dd_strs]

    # find xacro path
    curr_path = os.path.dirname(os.path.abspath(__file__))
    ind = curr_path.find('install')
    if (ind == -1) :
        node.get_logger().error('can not find workspace directory')
        return
    src_path = curr_path[:ind] + 'src'
    xacro_path = ''
    for dirpath, dirnames, filenames in os.walk(src_path):
        if dirpath.endswith('tmr_description'):
            xacro_path = dirpath + '/xacro'
            break
    if (xacro_path == ''):
        node.get_logger().error('can not find xacro directory')
        return

    xacro_name = '/macro.' + model + '.urdf.xacro'
    new_xacro_name = '/macro.' + new_model + '.urdf.xacro'

    file_in = xacro_path + xacro_name
    file_out = xacro_path + new_xacro_name

    link_tag = '<!--LinkDescription-->'
    link_head = '<?xml version=\'1.0\' encoding=\'UTF-8\'?>\n'
    link_start = '<data xmlns:xacro="http://wiki.ros.org/xacro">'
    link_end = '</data>'

    node.get_logger().info(file_in)

    fr = open(file_in, 'r')
    data_in = fr.read()
    fr.close()
    datas = data_in.split(link_tag)

    if len(datas) < 3:
        node.get_logger().info('invalid tmr...xacro')
        return

    link_data = link_start + datas[1] + link_end
    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh, '${prefix}')

    link_data = ET.tostring(root, encoding='UTF-8').decode('UTF-8')
    link_data = link_data.replace('ns0', 'xacro')
    link_data = link_data.replace(link_head, '', 1)
    link_data = link_data.replace(link_start, link_tag, 1)
    link_data = link_data.replace(link_end, link_tag, 1)

    data_out = datas[0] + link_data + datas[2]

    file_save = ''
    if replace:
        file_save = file_in
        node.get_logger().info('copy and rename origin xacro file')
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    node.get_logger().info(file_save)

    fw = open(file_save, 'w')
    fw.write(data_out)
    fw.close()

def main(args=None):
    try:
        _gen_urdf(args)
    except rclpy.exceptions.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
