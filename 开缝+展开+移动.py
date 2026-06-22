import Rhino.Geometry as rg
import math
from collections import deque

class PanelFace:
    """面板面片"""
    def __init__(self, brep_face, face_index):
        self.brep_face = brep_face
        self.index = face_index
        self.id = "F" + str(face_index)
        self.level = -1
        self.parent = None
        self.parent_edge = None
        self.children = []
        self.outer_edges = []

    def is_assigned(self):
        return self.level >= 0

class EdgeInfo:
    """边信息"""
    def __init__(self, edge_index, edge_curve, face_a, face_b=None):
        self.index = edge_index
        self.curve = edge_curve
        self.length = edge_curve.GetLength()
        self.face_a = face_a
        self.face_b = face_b
        self.status = "UNKNOWN"
        self.bend_point = None
        self.bend_angle = None
        self.bend_direction = None
        self.edge_type = None

    def get_other_face(self, this_face):
        return self.face_b if self.face_a == this_face else self.face_a

    def is_shared(self):
        return self.face_b is not None

class PanelUnfolder:
    """铝单板展开器 - 修正后的核心逻辑"""
    
    def __init__(self, brep):
        self.brep = brep
        self.faces = {}
        self.edges = {}
        self.base_face = None
        self.debug_messages = []
        self.bend_lines = []  # 3D折弯线
        self.cut_lines = []   # 3D切断线
        self.bend_points_3d = []  # 3D折弯点 (rg.Point3d)
        self.bend_angles = []   # 折弯类型字符串 (如 "正折90.0°")
        self.unfolded_textdots = []  # 展开后的TextDot (rg.TextDot)
        self.pieces = []
        # --- 新增：用于后续移动的映射字典 ---
        self.move_mapping_data = {}

    def add_debug(self, msg):
        self.debug_messages.append(msg)
        print(msg)

    def analyze_topology(self):
        """拓扑分析"""
        self.add_debug(f"【1】拓扑分析：{self.brep.Faces.Count}个面")
        
        for i in range(self.brep.Faces.Count):
            self.faces[i] = PanelFace(self.brep.Faces[i], i)
        
        for face_idx in range(self.brep.Faces.Count):
            face = self.brep.Faces[face_idx]
            for loop in face.Loops:
                for trim in loop.Trims:
                    if trim.Edge is None:
                        continue
                    edge_idx = trim.Edge.EdgeIndex
                    edge_curve = trim.Edge.DuplicateCurve()
                    
                    if edge_idx not in self.edges:
                        self.edges[edge_idx] = EdgeInfo(edge_idx, edge_curve, self.faces[face_idx])
                    else:
                        self.edges[edge_idx].face_b = self.faces[face_idx]
                    
                    if self.edges[edge_idx].face_a:
                        self.edges[edge_idx].face_a.outer_edges.append(self.edges[edge_idx])
                    if self.edges[edge_idx].face_b:
                        self.edges[edge_idx].face_b.outer_edges.append(self.edges[edge_idx])
        
        self.add_debug(f"  已建立 {len(self.edges)} 条边的拓扑关系")
        return self

    def find_base_face(self):
        """确定基面"""
        self.add_debug("【2】确定基面：选择面积最大的面")
        max_area = 0
        base_face = None
        
        for idx, face in self.faces.items():
            area = rg.AreaMassProperties.Compute(face.brep_face).Area
            if area > max_area:
                max_area = area
                base_face = face
        
        self.base_face = base_face or next(iter(self.faces.values()))
        self.base_face.level = 0
        self.base_face.id = "M"
        self.add_debug(f"  选定基面: F{self.base_face.index}, 面积={max_area:.2f}")
        return self

    def level_traversal(self):
        """层级遍历"""
        self.add_debug("【3】层级遍历：BFS遍历所有面")
        queue = deque([self.base_face])
        visited = set([self.base_face.index])
        
        while queue:
            current = queue.popleft()
            for edge in current.outer_edges:
                if not edge.is_shared():
                    continue
                neighbor = edge.get_other_face(current)
                if neighbor.index not in visited:
                    neighbor.level = current.level + 1
                    neighbor.parent = current
                    neighbor.parent_edge = edge
                    current.children.append(neighbor)
                    visited.add(neighbor.index)
                    queue.append(neighbor)
        
        self.add_debug(f"  完成层级遍历，共 {len(visited)} 个面")
        return self

    def resolve_multi_parent_conflicts(self):
        """解决多父冲突（关键修正：设置BrepEdge.Status）"""
        self.add_debug("【4】解决多父冲突：保留最长边，计算折弯角度")
        max_level = max(f.level for f in self.faces.values())
        
        for level in range(1, max_level + 1):
            level_faces = [f for f in self.faces.values() if f.level == level]
            for panel in level_faces:
                candidates = []
                for edge in panel.outer_edges:
                    if not edge.is_shared():
                        continue
                    other = edge.get_other_face(panel)
                    if other.level == panel.level - 1:
                        candidates.append((edge, other, edge.length))
                
                if not candidates:
                    continue
                
                best_edge, best_parent, best_len = max(candidates, key=lambda x: x[2])
                best_edge.status = "BEND"
                self.calculate_bend_angle(best_edge)
                
                # 3D折弯线
                self.bend_lines.append(best_edge.curve.DuplicateCurve())
                # 3D折弯点
                self.bend_points_3d.append(best_edge.bend_point)
                # 折弯类型
                self.bend_angles.append(f"{best_edge.bend_direction}{best_edge.bend_angle}°")
                
                for edge, _, _ in candidates:
                    if edge != best_edge:
                        edge.status = "CUT"
                        # 关键修正：设置BrepEdge.Status
                        self.brep.Edges[edge.index].Status = "CUT"
                        self.cut_lines.append(edge.curve.DuplicateCurve())
                
                panel.parent = best_parent
                panel.parent_edge = best_edge
                best_parent.children.append(panel)
        
        self.add_debug(f"  已处理 {len(self.bend_lines)} 条折弯边，{len(self.cut_lines)} 条切断边")
        return self

    def calculate_bend_angle(self, edge_info):
        """计算折弯角度（已验证）"""
        edge = self.brep.Edges[edge_info.index]
        adj_faces = edge.AdjacentFaces()
        if len(adj_faces) != 2:
            return
        
        faceA_idx, faceB_idx = adj_faces
        faceA = self.brep.Faces[faceA_idx]
        faceB = self.brep.Faces[faceB_idx]
        
        t = edge.Domain.ParameterAt(0.5)
        pt = edge.PointAt(t)
        edge_info.bend_point = pt
        
        successA, uA, vA = faceA.ClosestPoint(pt)
        successB, uB, vB = faceB.ClosestPoint(pt)
        if not successA or not successB:
            return
        
        nA = faceA.NormalAt(uA, vA)
        nB = faceB.NormalAt(uB, vB)
        nA.Unitize()
        nB.Unitize()
        
        tangent = self.get_standardized_edge_tangent(faceA, pt)
        if tangent is None:
            return
        
        cross = rg.Vector3d.CrossProduct(nA, nB)
        if cross.IsTiny(1e-3):
            return
        cross.Unitize()
        dot = cross * tangent
        
        tol = 1e-3
        if dot > tol:
            bend_type = "正折"
        elif dot < -tol:
            bend_type = "反折"
        else:
            bend_type = "正折"
        
        dot_n = nA * nB
        dot_n = max(-1.0, min(1.0, dot_n))
        angle_rad = math.acos(dot_n)
        bend_angle = round(180.0 - math.degrees(angle_rad), 1)
        
        edge_info.bend_angle = bend_angle
        edge_info.bend_direction = bend_type
        edge_info.edge_type = "凸边" if bend_type == "正折" else "凹边"

    def get_standardized_edge_tangent(self, ref_face, pt):
        """标准化切线方向"""
        outer_loop = None
        for loop in ref_face.Loops:
            if loop.LoopType == rg.BrepLoopType.Outer:
                outer_loop = loop
                break
        if not outer_loop:
            return None
        
        loop_3d_curve = outer_loop.To3dCurve()
        success, t_param = loop_3d_curve.ClosestPoint(pt)
        if not success:
            t_param = loop_3d_curve.Domain.ParameterAt(0.5)
        
        tangent = loop_3d_curve.TangentAt(t_param)
        tangent.Unitize()
        return tangent

    def break_same_level_edges(self):
        """处理同级边（关键修正：设置BrepEdge.Status）"""
        self.add_debug("【5】处理同级边：强制切断同级面之间的边")
        for edge in self.edges.values():
            if not edge.is_shared():
                continue
            faceA, faceB = edge.face_a, edge.face_b
            if faceA.level == faceB.level and faceA.level >= 0:
                edge.status = "CUT"
                # 关键修正：设置BrepEdge.Status
                self.brep.Edges[edge.index].Status = "CUT"
                self.cut_lines.append(edge.curve.DuplicateCurve())
        self.add_debug(f"  已切断 {len(self.cut_lines)} 条同级边")
        return self

    def unfold(self):
        """执行展开 - 使用UnjoinEdges预处理，再用Unroller展开每个片段"""
        self.add_debug("【7】执行展开：使用UnjoinEdges预处理，再用Unroller展开每个片段")
        
        # 1. 获取所有标记为"CUT"的边的索引
        cut_edge_indices = [edge.index for edge in self.edges.values() if edge.status == "CUT"]
        
        # 🔥 关键修复：检查是否需要切断
        if not cut_edge_indices:
            self.add_debug("  没有需要切断的边，直接使用原始Brep进行展开")
            unjoined_brep = self.brep  # 直接使用原始Brep
        else:
            # 2. 使用UnjoinEdges按CUT边将Brep分离成多个独立块
            separated_brep_blocks = self.brep.UnjoinEdges(cut_edge_indices)
            
            if not separated_brep_blocks:
                self.add_debug("  UnjoinEdges失败，没有返回任何Brep块")
                return None, []
            
            # 3. 取第一个分离后的Brep块作为Unroller的输入
            unjoined_brep = separated_brep_blocks[0]
            self.add_debug(f"  UnjoinEdges成功，得到 {len(separated_brep_blocks)} 个块，取第1个进行展开。")
        
        # 4. 创建Unroller实例，对已切断的Brep进行展开
        master_unroller = rg.Unroller(unjoined_brep)
        
        # 添加跟随几何体（标注点） - 关键修改：使用处理顺序索引
        following_dots = []
        for i, pt in enumerate(self.bend_points_3d):
            following_dots.append(rg.TextDot(str(i), pt))
        
        master_unroller.AddFollowingGeometry(following_dots)
        
        # 5. 执行展开
        result = master_unroller.PerformUnroll()
        
        if not result or not result[0]:
            self.add_debug("  展开失败：结果为空")
            return None, []
        
        # --- 修正：result[0] 是 Brep 数组，result[3] 是 TextDot 数组 ---
        flattened_brep_array = result[0]
        flattened_dot_list = result[3]

        # 6. 处理展开后的标注点 - 关键修改：直接按顺序匹配
        unrolled_dots_sorted = sorted(flattened_dot_list, key=lambda x: int(x.Text))
        for i, dot in enumerate(unrolled_dots_sorted):
            if i < len(self.bend_angles):
                text_dot = rg.TextDot(self.bend_angles[i], dot.Point)
                self.unfolded_textdots.append(text_dot)
        
        # --- 根据最终方案，构建 move_mapping_data ---
        # Unroller 保证了输出的顺序与输入时的顺序（按原始 brep face 索引）基本一致
        # 我们可以利用 self.faces.values() 的顺序与展开后 brep 的顺序进行匹配
        # 为了确保顺序对应，我们先创建一个按原始索引排序的列表
        ordered_face_objects = [self.faces[i] for i in sorted(self.faces.keys())]
        
        # 遍历展开后的 brep 数组
        for i, brep_part in enumerate(flattened_brep_array):
            # 由于顺序一致，flattened_brep_array[i] 对应 ordered_face_objects[i] 的展开结果
            if i >= len(ordered_face_objects):
                continue # 安全检查
            
            current_panel_face_obj = ordered_face_objects[i]
            level = current_panel_face_obj.level
            original_face_index = current_panel_face_obj.index
            
            # 初始化映射字典项
            mapping_entry = {
                'level': level,
                'parent_piece_index': -1, # 默认值
                'bend_angle': None,       # 默认值
                'original_face_index': original_face_index
            }

            # 如果不是主面，填充 parent_piece_index 和 bend_angle
            if level > 0:
                parent_original_obj = current_panel_face_obj.parent
                parent_original_idx = parent_original_obj.index
                
                # 查找 parent_original_idx 在 ordered_face_objects 列表中的位置 j
                # 这个位置 j 就是 parent_piece_index
                parent_piece_index = -1
                for j, p_face_obj in enumerate(ordered_face_objects):
                    if p_face_obj.index == parent_original_idx:
                        parent_piece_index = j
                        break
                
                # 获取折弯角度
                bend_angle_str = current_panel_face_obj.parent_edge.bend_direction + str(current_panel_face_obj.parent_edge.bend_angle)
                
                # 更新字典
                mapping_entry['parent_piece_index'] = parent_piece_index
                mapping_entry['bend_angle'] = bend_angle_str

            # 将条目存入总字典
            self.move_mapping_data[i] = mapping_entry

        return flattened_brep_array, self.move_mapping_data

    def generate_unfolded_pieces(self):
        """生成结果"""
        self.add_debug("【6】生成结果：调用展开功能")
        flattened_brep_array, move_mapping_data = self.unfold()
        if not flattened_brep_array:
            # --- 修改：返回7个空列表 ---
            return [], [], [], [], [], [], {}
        
        print(f"[映射数据] 生成的最终映射字典 (mapping_out):")
        for k, v in move_mapping_data.items():
             print(f"  pieces_out[{k}] -> {v}")
        
        # --- 修改：返回7个值 ---
        return flattened_brep_array, self.bend_lines, self.cut_lines, self.bend_points_3d, self.bend_angles, self.unfolded_textdots, move_mapping_data

def process_single_brep(brep):
    """处理单个Brep"""
    unfolder = PanelUnfolder(brep)
    unfolder.analyze_topology()
    unfolder.find_base_face()
    unfolder.level_traversal()
    unfolder.resolve_multi_parent_conflicts()
    unfolder.break_same_level_edges()
    # --- 修改：接收7个返回值 ---
    return unfolder.generate_unfolded_pieces()

def main(brep_list):
    """主函数"""
    if not hasattr(brep_list, '__iter__'):
        brep_list = [brep_list]
    
    all_pieces = []
    all_bend_lines = []
    all_cut_lines = []
    all_bend_points_3d = []
    all_bend_angles = []
    all_unfolded_textdots = []
    all_move_mapping_data = {} # 总映射字典
    
    for brep in brep_list:
        # --- 修改：接收7个返回值 ---
        pieces, bend_lines, cut_lines, bend_points_3d, bend_angles, unfolded_textdots, mapping_data = process_single_brep(brep)
        # --- 修改：pieces 现在是 Brep 数组，直接 extend ---
        all_pieces.extend(pieces)
        all_bend_lines.extend(bend_lines)
        all_cut_lines.extend(cut_lines)
        all_bend_points_3d.extend(bend_points_3d)
        all_bend_angles.extend(bend_angles)
        all_unfolded_textdots.extend(unfolded_textdots)
        # --- 新增：合并映射数据 ---
        # 需要更新映射中的索引，因为 pieces 被 extend 到了 all_pieces
        offset = len(all_pieces) - len(pieces) # 当前批次在 all_pieces 中的起始索引
        for local_idx, data in mapping_data.items():
            global_idx = offset + local_idx
            # 更新 parent_piece_index 以反映全局 all_pieces 的索引
            if data['parent_piece_index'] != -1:
                data['parent_piece_index'] = offset + data['parent_piece_index']
            all_move_mapping_data[global_idx] = data
    
    print(f"\n[主流程-最终映射] 合并后的总映射字典 (all_move_mapping_data):")
    for k, v in all_move_mapping_data.items():
         print(f"  all_pieces[{k}] -> {v}")
    return all_pieces, all_bend_lines, all_cut_lines, all_bend_points_3d, all_bend_angles, all_unfolded_textdots, all_move_mapping_data

# GH输出（七变量输出）
try:
    pieces_out, bend_lines_out, cut_lines_out, bend_points_out, bend_angles_out, unfolded_textdots_out, mapping_out = main(x)
except Exception as e:
    pieces_out, bend_lines_out, cut_lines_out, bend_points_out, bend_angles_out, unfolded_textdots_out, mapping_out = [], [], [], [], [], [], {}
    print(f"错误: {str(e)}")

a=pieces_out
b=bend_points_out
c=bend_angles_out
d=unfolded_textdots_out
# --- 新增输出变量 ---
e=mapping_out
print(e)

# --------------------------------------------------
# 新增功能：移动功能
# --------------------------------------------------



def find_2d_bend_line(brep1, brep2, tolerance=5.0):
    """
    在两个展开后的 Brep 面片之间查找2D折弯线。
    通过计算子面(brep1)各边的中点到父面(brep2)的距离来判断。

    参数:
        brep1 (Rhino.Geometry.Brep): 子面片 (例如 pieces_out[i])。
        brep2 (Rhino.Geometry.Brep): 父面片 (例如 pieces_out[j])。
        tolerance (float): 判断边是否重合的距离容差，默认为 5.0。

    返回:
        Rhino.Geometry.Curve: 找到的2D折弯线，如果未找到则返回 None。
    """
    closest_edge_curve = None
    min_distance = float('inf')

    # 遍历 brep1 (子面) 的所有边界边
    for edge in brep1.Edges:
        # 获取当前边的曲线
        current_curve = edge.DuplicateCurve()
        
        # 计算当前边的中点
        mid_param = current_curve.Domain.Mid
        mid_point = current_curve.PointAt(mid_param)
        
        # --- 核心：计算中点到父面 brep2 的距离 ---
        cp_result = brep2.ClosestPoint(mid_point)
        if cp_result: # 检查是否成功找到最近点
            distance = mid_point.DistanceTo(cp_result)
            
            # 更新最小距离和对应的边
            if distance < min_distance:
                min_distance = distance
                closest_edge_curve = current_curve

    # 检查找到的最小距离是否在容差范围内
    if min_distance <= tolerance and closest_edge_curve:
        return closest_edge_curve
    else:
        print(f"警告: 在两个面片间未找到距离小于 {tolerance} 的公共边，最小距离为 {min_distance:.4f}")
        return None


def apply_transform_to_brep_list(brep_list, transformations):
    """ 对一个 Brep 列表应用一系列变换。 """
    transformed_breps = []
    for i, brep in enumerate(brep_list):
        if i < len(transformations):
            xform = transformations[i]
            new_brep = brep.DuplicateBrep()
            new_brep.Transform(xform)
            transformed_breps.append(new_brep)
        else:
            # 如果没有对应的变换，则保持原样
            transformed_breps.append(brep)
    return transformed_breps

def move_pieces_based_on_mapping(pieces, mapping_data, thickness=3.0): # 增加厚度参数
    """
    根据 mapping_data 移动面片。
    逻辑：子面中心点 -> 折弯线垂足，计算扣除距离。
    正折：拉近（扣除正值）；反折：推开（扣除-1mm）。
    """
    if not pieces or not mapping_data:
        print("没有面片或映射数据，无法执行移动。")
        return pieces

    # 1. 为每个面片创建一个初始变换矩阵（单位矩阵）
    transforms = [rg.Transform.Identity for _ in pieces]

    # 2. 将映射数据按 Level 排序，确保先处理父面，再处理子面
    sorted_items = sorted(mapping_data.items(), key=lambda x: x[1]['level'])

    for child_index, data in sorted_items:
        parent_index = data['parent_piece_index']
        
        # 2.1 基面 (Level 0) 不动，跳过
        if parent_index == -1:
            continue

        # 2.2 获取父面和子面几何体
        parent_brep = pieces[parent_index]
        child_brep = pieces[child_index]

        # 2.3 查找 2D 折弯线
        bend_line_2d = find_2d_bend_line(child_brep, parent_brep)
        if not bend_line_2d:
            print(f"警告: 无法为面片 {child_index} 找到2D折弯线，跳过移动。")
            continue

        # 2.4 计算扣除量 (Deduction)
        bend_angle_str = data['bend_angle']
        if not bend_angle_str:
            continue

        deduction = 0.0
        is_positive = "正折" in bend_angle_str
        
        if is_positive:
            # 正折公式：1/TAN((角度/2)*PI/180) * 0.833 * 厚度 * 2
            try:
                angle_deg_str = bend_angle_str.replace("正折", "").replace("反折", "").replace("°", "")
                angle_deg = float(angle_deg_str)
                angle_rad = math.radians(angle_deg / 2.0)
                # 防止除以0
                if abs(angle_rad) > 1e-6:
                    # 0.833 是 K因子相关的经验系数
                    k_factor = 0.833 
                    deduction = (1.0 / math.tan(angle_rad)) * k_factor * thickness * 2.0
                else:
                    deduction = 0.0
            except:
                deduction = 0.0
        else:
            # 反折统一为 -1mm
            deduction = -1.0

        # 2.5 计算移动方向向量 (子面中心 -> 折弯线)
        # 获取子面2D中心点
        child_mp = rg.AreaMassProperties.Compute(child_brep)
        if not child_mp:
            continue
        child_center = child_mp.Centroid

        # 计算中心点在折弯线上的垂足 (最近点)
        success, t_param = bend_line_2d.ClosestPoint(child_center)
        if not success:
            continue
        foot_point = bend_line_2d.PointAt(t_param)

        # 向量：从子面中心 指向 折弯线 (这是“拉近”的方向)
        move_vector = foot_point - child_center
        if move_vector.IsZero:
            continue
            
        move_vector.Unitize() # 只保留方向

        # 2.6 计算最终位移
        # 将方向向量乘以扣除量
        final_vector = move_vector * deduction

        # 2.7 创建相对变换矩阵 (平移)
        relative_transform = rg.Transform.Translation(final_vector)

        # 2.8 累积变换：子面最终位置 = 父面最终位置 × 相对平移
        # 这样子面既会跟随父面移动，又会加上自己的扣除偏移
        parent_final_transform = transforms[parent_index]
        transforms[child_index] = parent_final_transform * relative_transform

    # 3. 应用所有计算好的变换矩阵
    moved_pieces = apply_transform_to_brep_list(pieces, transforms)
    print(f"移动完成，共处理了 {len(moved_pieces)} 个面片。")
    return moved_pieces

# --- 执行移动 ---
# 注意：调用时需要传入厚度参数，这里假设厚度为3.0，或者你需要从外部传入
oved_pieces = move_pieces_based_on_mapping(a, e, thickness=3.0) 

# --- 输出移动后的结果 ---
# f 可以是移动后的面片列表
f=oved_pieces
