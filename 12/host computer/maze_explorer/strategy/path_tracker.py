#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径跟踪模块
记录探索路径并计算返回路径
"""


class PathTracker:
    """路径跟踪器"""
    
    def __init__(self):
        """初始化路径跟踪器"""
        self.actions = []  # 动作序列：['forward', 'turn_right', ...]
        self.path = []     # 位置序列：[(x, y, facing), ...]
        
        # 当前状态
        self.current_pos = (0, 0)
        self.current_facing = 'north'  # north, east, south, west
        
        # 记录起点
        self.path.append((0, 0, 'north'))
    
    def record_action(self, action):
        """记录动作并更新当前状态
        
        Args:
            action: str, 'forward', 'turn_left', 'turn_right', 'turn_around'
        """
        if action is None:
            return
        
        # 记录动作
        self.actions.append(action)
        
        # 更新当前状态
        if action == 'forward':
            # 根据朝向更新位置
            dx, dy = self._get_direction_vector(self.current_facing)
            self.current_pos = (self.current_pos[0] + dx, 
                               self.current_pos[1] + dy)
        
        elif action == 'turn_left':
            self.current_facing = self._turn_left(self.current_facing)
        
        elif action == 'turn_right':
            self.current_facing = self._turn_right(self.current_facing)
        
        elif action == 'turn_around':
            self.current_facing = self._turn_around(self.current_facing)
        
        # 记录当前状态
        self.path.append((self.current_pos[0], 
                         self.current_pos[1], 
                         self.current_facing))
    
    def _get_direction_vector(self, facing):
        """获取朝向的方向向量
        
        Args:
            facing: str, 方向
        
        Returns:
            tuple: (dx, dy)
        """
        directions = {
            'north': (0, 1),
            'east': (1, 0),
            'south': (0, -1),
            'west': (-1, 0)
        }
        return directions.get(facing, (0, 0))
    
    def _turn_left(self, facing):
        """左转后的朝向"""
        turns = {
            'north': 'west',
            'west': 'south',
            'south': 'east',
            'east': 'north'
        }
        return turns.get(facing, facing)
    
    def _turn_right(self, facing):
        """右转后的朝向"""
        turns = {
            'north': 'east',
            'east': 'south',
            'south': 'west',
            'west': 'north'
        }
        return turns.get(facing, facing)
    
    def _turn_around(self, facing):
        """掉头后的朝向"""
        turns = {
            'north': 'south',
            'south': 'north',
            'east': 'west',
            'west': 'east'
        }
        return turns.get(facing, facing)
    
    def get_current_state(self):
        """获取当前状态
        
        Returns:
            dict: {
                'position': (x, y),
                'facing': str,
                'steps': int
            }
        """
        return {
            'position': self.current_pos,
            'facing': self.current_facing,
            'steps': len(self.actions)
        }
    
    def get_return_path(self):
        """计算返回起点的路径
        
        使用A*算法优化路径（当前版本：简化为直接反向）
        
        Returns:
            list: 返回动作序列 ['forward', 'turn_right', ...]
        """
        # 简化版：直接反向路径
        return self._reverse_path()
    
    def _reverse_path(self):
        """生成反向路径（简化版）
        
        Returns:
            list: 反向动作序列
        """
        # 当前朝向
        current_facing = self.current_facing
        
        # 反向动作序列
        reverse_actions = []
        
        # 从后往前遍历位置序列
        for i in range(len(self.path) - 1, 0, -1):
            prev_x, prev_y, _ = self.path[i-1]
            curr_x, curr_y, _ = self.path[i]
            
            # 计算需要前往的方向
            dx = prev_x - curr_x
            dy = prev_y - curr_y
            
            if dx == 0 and dy == 0:
                continue  # 原地转向，不需要移动
            
            # 确定目标朝向
            target_facing = None
            if dx == 0 and dy == 1:
                target_facing = 'north'
            elif dx == 0 and dy == -1:
                target_facing = 'south'
            elif dx == 1 and dy == 0:
                target_facing = 'east'
            elif dx == -1 and dy == 0:
                target_facing = 'west'
            
            if target_facing:
                # 添加转向动作
                turns = self._get_turn_actions(current_facing, target_facing)
                reverse_actions.extend(turns)
                current_facing = target_facing
                
                # 添加前进动作
                reverse_actions.append('forward')
        
        return reverse_actions
    
    def _get_turn_actions(self, from_facing, to_facing):
        """计算从一个朝向转到另一个朝向需要的动作
        
        Args:
            from_facing: str, 起始朝向
            to_facing: str, 目标朝向
        
        Returns:
            list: 转向动作序列
        """
        if from_facing == to_facing:
            return []
        
        facings = ['north', 'east', 'south', 'west']
        from_idx = facings.index(from_facing)
        to_idx = facings.index(to_facing)
        
        # 计算最短转向
        diff = (to_idx - from_idx) % 4
        
        if diff == 1:
            return ['turn_right']
        elif diff == 2:
            return ['turn_right', 'turn_right']  # 或 ['turn_around']
        elif diff == 3:
            return ['turn_left']
        
        return []

