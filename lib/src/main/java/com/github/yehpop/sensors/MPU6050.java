// "SPDX-License-Identifier:	GPL-3.0-or-later"
/* 
 * This file is part of nf-sensors.
 * Copyright (C) 2023 Yüşa Furkan Erenci, Tuna Gül
 */
package com.github.yehpop.nfsensors;

import java.nio.ByteBuffer;
import java.lang.Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.I2C;


/**
 * Class to use MPU6050 to get rotation and angle.
 */
public class MPU6050 implements Gyro {
    /**
     * Enum for storing specific axis.
     * This stored axis is later used when specifying the axis that should measure the yaw.
     */
    public enum YawAxis {
      X,
      Y,
      Z
    }

    private ByteBuffer m_readBuffer = ByteBuffer.allocate(14);
    private MPUData m_configuredState = new MPUData();
    private byte m_I2CAddress;
    private I2C m_i2c;
    private YawAxis m_yaw;

    /**
     * Type class for storing the data recieved from MPU6050
     * 
     * MPUData.m_accX: X axis data of accelerometer
     * MPUData.m_accY: Y axis data of accelerometer
     * MPUData.m_accZ: Z axis data of accelerometer
     * MPUData.m_temperature: Temperature data
     * MPUData.m_gyroX: X axis data of gyro
     * MPUData.m_gyroY: Y axis data of gyro
     * MPUData.m_gyroZ: Z axis data of gyro
     */
    private static class MPUData {
      public short m_accX, m_accY, m_accZ, m_gyroX, m_gyroY, m_gyroZ;
      private short m_temperature;
      
  
      public MPUData() {
        m_accX = 0;
        m_accY = 0;
        m_accZ = 0;
        m_temperature = 0;
        m_gyroX = 0;
        m_gyroY = 0;
        m_gyroZ = 0;
      }
  
      public MPUData(ByteBuffer MPUReading) {
        m_accX = MPUReading.getShort(0);
        m_accY = MPUReading.getShort(2);
        m_accZ = MPUReading.getShort(4);
        m_temperature = MPUReading.getShort(6);
        m_gyroX = MPUReading.getShort(8);
        m_gyroY = MPUReading.getShort(10);
        m_gyroZ = MPUReading.getShort(12);
      }
  
      public void subtract(MPUData other) {
        m_accX -= other.m_accX;
        m_accY -= other.m_accY;
        m_accZ -= other.m_accZ;
        // temprature için offset yok
        // m_temperature -= other.m_temperature;
        m_gyroX -= other.m_gyroX;
        m_gyroY -= other.m_gyroY;
        m_gyroZ -= other.m_gyroZ;
      }
  
      public double getTemperature() {
        // Sıcaklığı hesaplamak için biraz işlem
        return m_temperature/340. + 36.53;
      }
    }

    /** 
     * @param I2CAddress i2c address of the mpu6050. (0x68)
     */
    public MPU6050(byte I2CAddress) {
      m_I2CAddress = I2CAddress;
      m_i2c = new I2C(I2C.Port.kMXP, m_I2CAddress);
      m_i2c.write(0x6B, 0);
      m_yaw = YawAxis.X;
    }

    /** 
     * @param I2CAddress i2c address of the mpu6050. (0x68)
     * 
     * @param yaw the axis that measures the yaw. X axis by default. can be YawAxis.X, YawAxis.Y or YawAxis.Z
     */
    public MPU6050(byte I2CAddress, final YawAxis yaw) {
      m_I2CAddress = I2CAddress;
      m_i2c = new I2C(I2C.Port.kMXP, m_I2CAddress);
      m_i2c.write(0x6B, 0);
      m_yaw = yaw;
    }
    
    private double _map(double x, double inMin, double inMax, double outMin, double outMax) {
      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
    
    /**
     * @return The address of the device in byte for I2C
     */
    public byte getI2CAddress() {
      return m_I2CAddress;
    }
    
    /**
     * Gets accelerometer gyro and temperature data from the sensor.
     * 
     * @return data recieved from the MPU6050 in type MPUData
     */
    private MPUData getMPUData() {
      m_i2c.read(0x3B, 14, m_readBuffer);
      MPUData readData = new MPUData(m_readBuffer);
      readData.subtract(m_configuredState);
      return readData;
    }
    
    public void calibrate() {} 
    
    @Override
    public void reset() {
      m_configuredState = getMPUData();
    }
    
    /**
     * @return accumulated gyro angle on selected axis in degrees.
     */
    @Override
    public double getAngle() {
      switch (m_yaw) {
        case X:
          return getAngleX();
        case Y:
          return getAngleY();
        case Z:
          return getAngleZ();
        default:
          return 0.0;
      }
    }
    
    /**
     * @return angular rate of gyro's selected axis in degrees per second.
     */
    @Override
    public double getRate() {
      switch (m_yaw) {
        case X:
          return getRateX();
        case Y:
          return getRateY();
        case Z:
          return getRateZ();
        default:
          return 0.0;
      }
    }
    
    /**
     * Free the I2C Port used for MPU6050
     */
    @Override
    public void close() {
      if (m_i2c != null) {
        m_i2c.close();  
        m_i2c = null;
      }
      System.out.println("Cleaned after I2C Port.");
    }

    /**
     * 
     * @return all axises current angle in a list of doubles.
     */
    public double[] getAllAngles() {
      MPUData data = getMPUData();
      double ax = _map(data.m_accX, 265, 402, -90, 90);
      double ay = _map(data.m_accY, 265, 402, -90, 90);
      double az = _map(data.m_accZ, 265, 402, -90, 90);
  
      ax = Math.atan2(-ay, -az) + Math.PI;
      ay = Math.atan2(-ax, -az) + Math.PI;
      az = Math.atan2(-ay, -ax) + Math.PI;
    
      return new double[] { ax, ay, az };
    }
  
    /**
     * @return accumulated gyro angle of x axis in degrees
     */
    public double getAngleX() {
      MPUData data = getMPUData();
      double ay = _map(data.m_accY, 265, 402, -90, 90);
      double az = _map(data.m_accZ, 265, 402, -90, 90);
  
      return Math.atan2(-ay, -az) + Math.PI;
    }

    /**
     * @return accumulated gyro angle of y axis in degrees
     */
    public double getAngleY() {
      MPUData data = getMPUData();
      double ax = _map(data.m_accX, 265, 402, -90, 90);
      double az = _map(data.m_accZ, 265, 402, -90, 90);
  
      return Math.atan2(-ax, -az) + Math.PI;
    }
  
    /**
     * @return accumulated gyro angle of z axis in degrees
     */
    public double getAngleZ() {
      MPUData data = getMPUData();
      double ax = _map(data.m_accX, 265, 402, -90, 90);
      double ay = _map(data.m_accY, 265, 402, -90, 90);
  
      return Math.atan2(-ay, -ax) + Math.PI;
    }

    /**
     * @return angular rate of gyro's x axis in degrees per second.
     */
    public double getRateX() {
      MPUData data = getMPUData();
      return data.m_gyroX;
    }

    /**
     * @return angular rate of gyro's y axis in degrees per second.
     */
    public double getRateY() {
      MPUData data = getMPUData();
      return data.m_gyroY;
    }

    /**
     * @return angular rate of gyro's z axis in degrees per second.
     */
    public double getRateZ() {
      MPUData data = getMPUData();
      return data.m_gyroZ;
    }
    
    /**
     * @return rotation from MPU6050's X axis as wpilib's Rotation2d
     */
    public Rotation2d getRotationX() {
      MPUData data = getMPUData();
      double ay = _map(data.m_accY, 265, 402, -90, 90);
      double az = _map(data.m_accZ, 265, 402, -90, 90);
  
      return new Rotation2d(Math.atan2(-ay, -az) + Math.PI);
    }
  
    /**
     * @return rotation from MPU6050's Y axis as wpilib's Rotation2d
     */
    public Rotation2d getRotationY() {
      MPUData data = getMPUData();
      double ax = _map(data.m_accX, 265, 402, -90, 90);
      double az = _map(data.m_accZ, 265, 402, -90, 90);
  
      return new Rotation2d(Math.atan2(-ax, -az) + Math.PI);
    }
  
    /**
     * @return rotation from MPU6050's Z axis as wpilib's Rotation2d
     */
    public Rotation2d getRotationZ() {
      MPUData data = getMPUData();
      double ax = _map(data.m_accX, 265, 402, -90, 90);
      double ay = _map(data.m_accY, 265, 402, -90, 90);
  
      return new Rotation2d(Math.atan2(-ay, -ax) + Math.PI);
    }
  
    /**
     * Prints all accumulated gyro angles on all axises.
     */
    public void printAngles() {
      System.out.print(Math.toDegrees(getAngleX()));
      System.out.print(", \t");
      System.out.print(Math.toDegrees(getAngleY()));
      System.out.print(", \t");
      System.out.print(Math.toDegrees(getAngleX()));
      System.out.println();
    }
    
    /**
     * Prints accelerometer data from the sensor.
     */
    public void printAccData() {
      MPUData data = getMPUData();
      System.out.print("Acc X: ");
      System.out.print(data.m_accX);
      System.out.print(" Acc Y: ");
      System.out.print(data.m_accY);
      System.out.print(" Acc Z: ");
      System.out.println(data.m_accZ);
    }
    
    /**
     * Prints all data recieved from the sensor.
     */
    public void printAllData() {
      MPUData data = getMPUData();
      System.out.print("Acc: (");
      System.out.print(data.m_accX/16384.);
      System.out.print(", ");
      System.out.print(data.m_accY/16384.);
      System.out.print(", ");
      System.out.print(data.m_accZ/16384.);
      System.out.print(");  ");
      
      System.out.print(data.getTemperature());
      System.out.print("  ");
  
      System.out.print("Gyro: (");
      System.out.print(data.m_gyroX/131.);
      System.out.print(", ");
      System.out.print(data.m_gyroY/131.);
      System.out.print(", ");
      System.out.print(data.m_gyroZ/131.);
      System.out.println("); ");
    }
  
    /**
     * Prints temperature data.
     */
    public void printTemperature() {
      MPUData data = getMPUData();
      System.out.println(data.getTemperature());
    }
  
    /**
     * Prints all data from MPU6050 as a gyroscope.
     * Used for debug.
     */
    public void printGyroData() {
      MPUData data = getMPUData();
      System.out.print("Gyro X: ");
      System.out.print(data.m_gyroX);
      System.out.print(" Gyro Y: ");
      System.out.print(data.m_gyroY);
      System.out.print(" Gyro Z: ");
      System.out.println(data.m_gyroZ);
    }
  }
