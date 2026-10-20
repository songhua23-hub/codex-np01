"""Grasshopper Script Instance"""
import System
import Rhino
import Grasshopper

import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import math
from collections import deque
from System.Collections.Generic import IEnumerable



thickness = 3.0    #请注意输入brep的板统一厚度


class MyComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, x: System.Collections.Generic.List[Rhino.Geometry.Brep]):
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
                self.angle_deg = None
                self.avoidance_C = None
                self.avoidance_face_index = None
        
            def get_other_face(self, this_face):
                return self.face_b if self.face_a == this_face else self.face_a
        
            def is_shared(self):
                return self.face_b is not None
        
        class PanelUnfolder:
            """铝单板展开器"""
            def __init__(self, brep):
                self.brep = brep
                self.faces = {}
                self.edges = {}
                self.base_face = None
                self.debug_messages = []
                self.bend_lines = []
                self.cut_lines = []
                self.bend_points_3d = []
                self.bend_angles = []
                self.unfolded_textdots = []
                self.pieces = []
                self.move_mapping_data = {}
                self.cut_mid_2d = {}   
        
            def add_debug(self, msg):
                self.debug_messages.append(msg)
                print(msg)
        
            def analyze_topology(self):
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
        
            def _compute_avoidance_params(self, edge, T, weld_gap=0.5):
                if not edge.is_shared():
                    return
                t = edge.curve.Domain.Mid
                pt = edge.curve.PointAt(t)
                successA, uA, vA = edge.face_a.brep_face.ClosestPoint(pt)
                successB, uB, vB = edge.face_b.brep_face.ClosestPoint(pt)
                if not successA or not successB:
                    return
                nA = edge.face_a.brep_face.NormalAt(uA, vA); nA.Unitize()
                nB = edge.face_b.brep_face.NormalAt(uB, vB); nB.Unitize()
                dot = max(-1.0, min(1.0, nA * nB))
                alpha_deg = math.degrees(math.acos(dot))
        
                if alpha_deg < 1.0 or alpha_deg > 179.0:
                    real_angle = 0.0
                else:
                    real_angle = alpha_deg if alpha_deg <= 90.0 else 180.0 - alpha_deg
        
                if real_angle <= 90.0 and real_angle > 1.0:
                    C = T / math.tan(math.radians(real_angle / 2.0)) + weld_gap
                else:
                    C = T * math.cos(math.radians(real_angle - 90.0)) + weld_gap
                    
                edge.angle_deg = real_angle
                edge.avoidance_C = C
                edge.avoidance_face_index = edge.face_b.index
                print("  [避让计算] 边{}: 法向夹角={:.1f}°, 真实折弯角={:.1f}°, C={:.2f}mm".format(edge.index, alpha_deg, real_angle, C))
        
            def resolve_multi_parent_conflicts(self):
                self.add_debug("【4】解决多父冲突")
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
                        
                        self.bend_lines.append(best_edge.curve.DuplicateCurve())
                        self.bend_points_3d.append(best_edge.bend_point)
                        self.bend_angles.append(f"{best_edge.bend_direction}{best_edge.bend_angle}°")
                        
                        for edge, _, _ in candidates:
                            if edge != best_edge:
                                edge.status = "CUT"
                                self.brep.Edges[edge.index].Status = "CUT"
                                self.cut_lines.append(edge.curve.DuplicateCurve())
                                self._compute_avoidance_params(edge, T=thickness, weld_gap=0.5)
                        
                        panel.parent = best_parent
                        panel.parent_edge = best_edge
                        best_parent.children.append(panel)
                self.add_debug(f"  已处理 {len(self.bend_lines)} 条折弯边，{len(self.cut_lines)} 条切断边")
                return self
        
            def calculate_bend_angle(self, edge_info):
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
                nA = faceA.NormalAt(uA, vA); nA.Unitize()
                nB = faceB.NormalAt(uB, vB); nB.Unitize()
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
                self.add_debug("【5】处理同级边")
                for edge in self.edges.values():
                    if not edge.is_shared():
                        continue
                    faceA, faceB = edge.face_a, edge.face_b
                    if faceA.level == faceB.level and faceA.level >= 0:
                        edge.status = "CUT"
                        self.brep.Edges[edge.index].Status = "CUT"
                        self.cut_lines.append(edge.curve.DuplicateCurve())
                        self._compute_avoidance_params(edge, T=thickness, weld_gap=0.5)
                self.add_debug(f"  已切断 {len(self.cut_lines)} 条同级边")
                return self
        
            def unfold(self):
                self.add_debug("【7】执行展开")
                cut_edge_indices = [edge.index for edge in self.edges.values() if edge.status == "CUT"]
                if not cut_edge_indices:
                    self.add_debug("  没有需要切断的边，直接使用原始Brep进行展开")
                    unjoined_brep = self.brep
                else:
                    separated_brep_blocks = self.brep.UnjoinEdges(cut_edge_indices)
                    if not separated_brep_blocks:
                        self.add_debug("  UnjoinEdges失败")
                        return None, []
                    unjoined_brep = separated_brep_blocks[0]
                    self.add_debug(f"  UnjoinEdges成功，得到 {len(separated_brep_blocks)} 个块。")
                
                master_unroller = rg.Unroller(unjoined_brep)
                following_dots = []
                for i, pt in enumerate(self.bend_points_3d):
                    following_dots.append(rg.TextDot(str(i), pt))
                for ce in self.edges.values():
                    if ce.status == "CUT":
                        mid = ce.curve.PointAt(ce.curve.Domain.Mid)
                        following_dots.append(rg.TextDot("CUT_" + str(ce.index), mid))
                
                master_unroller.AddFollowingGeometry(following_dots)
                result = master_unroller.PerformUnroll()
                if not result or not result[0]:
                    self.add_debug("  展开失败：结果为空")
                    return None, []
                
                flattened_brep_array = result[0]
                flattened_dot_list = result[3]
        
                self.cut_mid_2d = {d.Text: d.Point for d in flattened_dot_list if d.Text.startswith("CUT_")}
                unrolled_dots_sorted = sorted([d for d in flattened_dot_list if not d.Text.startswith("CUT_")], key=lambda x: int(x.Text))
                
                for i, dot in enumerate(unrolled_dots_sorted):
                    if i < len(self.bend_angles):
                        text_dot = rg.TextDot(self.bend_angles[i], dot.Point)
                        self.unfolded_textdots.append(text_dot)
                
                ordered_face_objects = [self.faces[i] for i in sorted(self.faces.keys())]
                for i, brep_part in enumerate(flattened_brep_array):
                    if i >= len(ordered_face_objects):
                        continue
                    current_panel_face_obj = ordered_face_objects[i]
                    level = current_panel_face_obj.level
                    original_face_index = current_panel_face_obj.index
                    mapping_entry = {
                        'level': level,
                        'parent_piece_index': -1,
                        'bend_angle': None,
                        'original_face_index': original_face_index
                    }
                    if level > 0:
                        parent_original_obj = current_panel_face_obj.parent
                        parent_original_idx = parent_original_obj.index
                        parent_piece_index = -1
                        for j, p_face_obj in enumerate(ordered_face_objects):
                            if p_face_obj.index == parent_original_idx:
                                parent_piece_index = j
                                break
                        bend_angle_str = current_panel_face_obj.parent_edge.bend_direction + str(current_panel_face_obj.parent_edge.bend_angle)
                        mapping_entry['parent_piece_index'] = parent_piece_index
                        mapping_entry['bend_angle'] = bend_angle_str
                    self.move_mapping_data[i] = mapping_entry
        
                return flattened_brep_array, self.move_mapping_data
        
            def generate_unfolded_pieces(self):
                self.add_debug("【6】生成结果")
                flattened_brep_array, move_mapping_data = self.unfold()
                if not flattened_brep_array:
                    return [], [], [], [], [], [], {}
                try:
                    flattened_brep_array, move_mapping_data = apply_avoidance(
                        flattened_brep_array, move_mapping_data,
                        self.edges, self.faces, self.cut_mid_2d, T=thickness, weld_gap=0.5)
                except Exception as ex:
                    self.add_debug("  避让模块异常: " + str(ex))
                return flattened_brep_array, self.bend_lines, self.cut_lines, self.bend_points_3d, self.bend_angles, self.unfolded_textdots, move_mapping_data

        # 【注意：必须放在类中，否则找不到函数】
        def apply_avoidance(pieces, mapping, edges, faces, cut_mid_2d, T=2.0, weld_gap=0.5, tolerance=0.01):
            if not pieces or not mapping:
                return pieces, mapping

            face_to_piece = {}
            for pidx, info in mapping.items():
                ofi = info.get('original_face_index', None)
                if ofi is not None:
                    face_to_piece[ofi] = pidx

            cut_edges = [e for e in edges.values() if e.status == "CUT"]
            processed_boundaries = set()

            for ce in cut_edges:
                target_face_idx = ce.avoidance_face_index
                C = ce.avoidance_C
                if target_face_idx is None or C is None:
                    continue

                piece_idx = face_to_piece.get(target_face_idx, None)
                if piece_idx is None:
                    continue

                piece = pieces[piece_idx]
                if piece is None or piece.Faces.Count == 0:
                    continue

                ok, plane = piece.Faces[0].TryGetPlane(0.1)
                if not ok:
                    continue

                mp = rg.AreaMassProperties.Compute(piece)
                if not mp: continue
                center = mp.Centroid

                key = "CUT_" + str(ce.index)
                if key not in cut_mid_2d:
                    continue
                p_mid = cut_mid_2d[key]

                candidates = []
                for edge in piece.Edges:
                    c = edge.DuplicateCurve()
                    ok, t = c.ClosestPoint(p_mid)
                    if not ok: continue
                    d = p_mid.DistanceTo(c.PointAt(t))
                    L2d = c.GetLength()
                    len_err = abs(L2d - ce.length) / max(ce.length, 1e-6)
                    if len_err < 0.05: 
                        candidates.append((c, d))
                
                if not candidates:
                    print("警告: CUT边{}长度匹配失败，找不到对应2D边界".format(ce.index))
                    continue
                
                candidates.sort(key=lambda x: x[1])
                boundary = candidates[0][0]

                b_mid = boundary.PointAt(boundary.Domain.Mid)
                check_key = (piece_idx, round(b_mid.X, 1), round(b_mid.Y, 1), round(b_mid.Z, 1))
                if check_key in processed_boundaries:
                    continue

                correct_offset = None
                for sign in (-1.0, 1.0):
                    offs = boundary.Offset(plane, sign * C, tolerance, rg.CurveOffsetCornerStyle.Sharp)
                    if not offs: continue
                    for oc in offs:
                        mid = oc.PointAt(oc.Domain.Mid)
                        ok_in, u, v = piece.Faces[0].ClosestPoint(mid)
                        if not ok_in: continue
                        if piece.Faces[0].IsPointOnFace(u, v) != rg.PointFaceRelation.Interior:
                            continue
                        
                        dist_b = b_mid.DistanceTo(center)
                        dist_oc = mid.DistanceTo(center)
                        if dist_oc < dist_b: 
                            correct_offset = oc
                            break
                    if correct_offset: break

                if correct_offset is None:
                    print("警告: CUT边{}偏移方向判断失败，跳过".format(ce.index))
                    continue

                # 【极其简单的延长法】提取起点和终点，沿着方向两头拉长 10mm
                p_start = correct_offset.PointAt(correct_offset.Domain.Min)
                p_end = correct_offset.PointAt(correct_offset.Domain.Max)
                vec = p_end - p_start
                if vec.Length > 0.001:
                    vec.Unitize()
                    p_start = p_start - vec * 10.0
                    p_end = p_end + vec * 10.0
                
                # 重新画一根长了 20mm 的直线
                ext_curve = rg.LineCurve(p_start, p_end)

                # 用这根长线去切面片
                spl = piece.Split.Overloads[IEnumerable[rg.Curve], System.Double]([ext_curve], tolerance)
                
                if not spl or len(spl) < 2:
                    continue
                
                kept = None
                for s in spl:
                    ok2, u, v = s.Faces[0].ClosestPoint(center)
                    if ok2 and s.Faces[0].IsPointOnFace(u, v) == rg.PointFaceRelation.Interior:
                        kept = s
                        break
                if kept is not None:
                    pieces[piece_idx] = kept
                    processed_boundaries.add(check_key)
                    print("  [避让成功] 面片{} 已切缝，C={:.2f}，切边坐标({:.1f}, {:.1f})".format(piece_idx, C, b_mid.X, b_mid.Y))

            return pieces, mapping


        # ==================================================
        # 主流程
        # ==================================================
        def process_single_brep(brep):
            unfolder = PanelUnfolder(brep)
            unfolder.analyze_topology()
            unfolder.find_base_face()
            unfolder.level_traversal()
            unfolder.resolve_multi_parent_conflicts()
            unfolder.break_same_level_edges()
            return unfolder.generate_unfolded_pieces()

        def main(brep_list):
            if not hasattr(brep_list, '__iter__'):
                brep_list = [brep_list]
            
            all_pieces = []
            all_bend_lines = []
            all_cut_lines = []
            all_bend_points_3d = []
            all_bend_angles = []
            all_unfolded_textdots = []
            all_move_mapping_data = {}
            
            for brep in brep_list:
                pieces, bend_lines, cut_lines, bend_points_3d, bend_angles, unfolded_textdots, mapping_data = process_single_brep(brep)
                all_pieces.extend(pieces)
                all_bend_lines.extend(bend_lines)
                all_cut_lines.extend(cut_lines)
                all_bend_points_3d.extend(bend_points_3d)
                all_bend_angles.extend(bend_angles)
                all_unfolded_textdots.extend(unfolded_textdots)
                offset = len(all_pieces) - len(pieces)
                for local_idx, data in mapping_data.items():
                    global_idx = offset + local_idx
                    if data['parent_piece_index'] != -1:
                        data['parent_piece_index'] = offset + data['parent_piece_index']
                    all_move_mapping_data[global_idx] = data
            
            return all_pieces, all_bend_lines, all_cut_lines, all_bend_points_3d, all_bend_angles, all_unfolded_textdots, all_move_mapping_data


        # GH输出
        


        try:
            pieces_out, bend_lines_out, cut_lines_out, bend_points_out, bend_angles_out, unfolded_textdots_out, mapping_out = main(x)
        except Exception as e:
            pieces_out, bend_lines_out, cut_lines_out, bend_points_out, bend_angles_out, unfolded_textdots_out, mapping_out = [], [], [], [], [], [], {}
            print(f"错误: {str(e)}")

        a = pieces_out
        b = bend_points_out
        c = bend_angles_out
        d = unfolded_textdots_out
        e = mapping_out


        # --------------------------------------------------
        # 移动功能
        # --------------------------------------------------
        def find_2d_bend_line(brep1, brep2, tolerance=5.0):
            closest_edge_curve = None
            min_distance = float('inf')
            for edge in brep1.Edges:
                current_curve = edge.DuplicateCurve()
                mid_param = current_curve.Domain.Mid
                mid_point = current_curve.PointAt(mid_param)
                cp_result = brep2.ClosestPoint(mid_point)
                if cp_result:
                    distance = mid_point.DistanceTo(cp_result)
                    if distance < min_distance:
                        min_distance = distance
                        closest_edge_curve = current_curve
            if min_distance <= tolerance and closest_edge_curve:
                return closest_edge_curve
            return None

        def apply_transform_to_brep_list(brep_list, transformations):
            transformed_breps = []
            for i, brep in enumerate(brep_list):
                if i < len(transformations):
                    xform = transformations[i]
                    new_brep = brep.DuplicateBrep()
                    new_brep.Transform(xform)
                    transformed_breps.append(new_brep)
                else:
                    transformed_breps.append(brep)
            return transformed_breps

        def move_pieces_based_on_mapping(pieces, mapping_data, thickness=2.0):
            if not pieces or not mapping_data:
                return pieces

            transforms = [rg.Transform.Identity for _ in pieces]
            sorted_items = sorted(mapping_data.items(), key=lambda x: x[1]['level'])

            for child_index, data in sorted_items:
                parent_index = data['parent_piece_index']
                if parent_index == -1:
                    continue

                parent_brep = pieces[parent_index]
                child_brep = pieces[child_index]

                bend_line_2d = find_2d_bend_line(child_brep, parent_brep)
                if not bend_line_2d:
                    continue

                bend_angle_str = data['bend_angle']
                if not bend_angle_str:
                    continue

                deduction = 0.0
                is_positive = "正折" in bend_angle_str
                
                if is_positive:
                    try:
                        angle_deg_str = bend_angle_str.replace("正折", "").replace("反折", "").replace("°", "")
                        angle_deg = float(angle_deg_str)
                        angle_rad = math.radians(angle_deg / 2.0)
                        if abs(angle_rad) > 1e-6:
                            k_factor = 0.833
                            deduction = (1.0 / math.tan(angle_rad)) * k_factor * thickness * 2.0
                        else:
                            deduction = 0.0
                    except:
                        deduction = 0.0
                else:
                    deduction = -1.0

                child_mp = rg.AreaMassProperties.Compute(child_brep)
                if not child_mp:
                    continue
                child_center = child_mp.Centroid

                success, t_param = bend_line_2d.ClosestPoint(child_center)
                if not success:
                    continue
                foot_point = bend_line_2d.PointAt(t_param)

                move_vector = foot_point - child_center
                if move_vector.IsZero:
                    continue
                move_vector.Unitize()

                final_vector = move_vector * deduction
                relative_transform = rg.Transform.Translation(final_vector)
                parent_final_transform = transforms[parent_index]
                transforms[child_index] = parent_final_transform * relative_transform

            moved_pieces = apply_transform_to_brep_list(pieces, transforms)
            return moved_pieces

        # 执行移动
        f = move_pieces_based_on_mapping(a, e, thickness)
        
        return a, b, c, d, e, f