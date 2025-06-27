-- 创建数据库 
CREATE DATABASE IF NOT EXISTS bus CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;

-- 使用数据库
USE bus;

-- 创建车辆数据表
CREATE TABLE IF NOT EXISTS `vehicle_data` (
    `id` INT AUTO_INCREMENT PRIMARY KEY COMMENT '自增主键',
    `timestamp` TIMESTAMP DEFAULT CURRENT_TIMESTAMP COMMENT '数据记录时间',
    `engine_rpm` INT COMMENT '发动机转速 (rpm)',
    `vehicle_speed` INT COMMENT '车辆速度 (km/h)', 
    `steer_angle` INT COMMENT '方向盘转向角度',
    `throttle_position` INT COMMENT '油门踏板位置 (%)', 
    `brake_pressure` INT COMMENT '刹车压力',
    `transmission_gear` INT COMMENT '变速箱档位',
    `vehicle_state` INT COMMENT '车辆状态码'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COMMENT='车辆CAN总线数据记录表';

-- 创建索引以提高查询性能
CREATE INDEX idx_timestamp ON `vehicle_data` (timestamp);

-- 可选：创建存储过程，用于数据分析
DELIMITER //
CREATE PROCEDURE GetVehicleDataSummary(IN start_time TIMESTAMP, IN end_time TIMESTAMP)
BEGIN
    SELECT 
        MIN(engine_rpm) AS min_rpm,
        MAX(engine_rpm) AS max_rpm,
        AVG(engine_rpm) AS avg_rpm,
        MIN(vehicle_speed) AS min_speed,
        MAX(vehicle_speed) AS max_speed,
        AVG(vehicle_speed) AS avg_speed
    FROM vehicle_data
    WHERE timestamp BETWEEN start_time AND end_time;
END //
DELIMITER ;
