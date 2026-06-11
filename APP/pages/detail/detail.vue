<template>
	<!-- 整体布局 -->
	<view class="wrap">
		<!-- 系统状态区域 -->
		<view class="status-area">
			<view class="status-title">系统状态</view>
			<view class="status-content" :class="SystemStatus === 0 ? 'status-ok' : 'status-err'">
				{{ statusText }}
			</view>
		</view>

		<!-- 设备区域 -->
		<view class="dev-area">

			<!-- 设备卡片 - 蜂鸣器 -->
			<view class="dev-cart">
				<view>
					<view class="dev-name">蜂鸣器</view>
					<image class="dev-logo" src="../../static/buzzer.png" mode=""></image>
				</view>
				<switch :checked="Buzzer" @change="onBuzzerSwitch" color="#2b9939" />
			</view>

			<!-- 设备卡片 - 车锁 -->
			<view class="dev-cart">
				<view>
					<view class="dev-name">车锁</view>
					<image class="dev-logo" src="../../static/lamp.png" mode=""></image>
				</view>
				<switch :checked="CarLock" @change="onCarLockSwitch" color="#2b9939" />
			</view>

		</view>

		<!-- 模拟按键区域 -->
		<view class="btn-area">
			<view class="btn-title">模拟按键</view>
			<view class="btn-row">
				<view class="btn-cart" :class="Brake ? 'btn-active' : ''" @touchstart="onBrakeStart" @touchend="onBrakeEnd">
					<view class="btn-name">刹车</view>
				</view>
				<view class="btn-cart" :class="ParkingBrake ? 'btn-active' : ''" @touchstart="onParkingBrakeStart" @touchend="onParkingBrakeEnd">
					<view class="btn-name">手刹</view>
				</view>
			</view>
		</view>

		<!-- 距离显示 -->
		<view class="distance-area">
			<view class="distance-title">车钥匙距离</view>
			<view class="distance-value">{{ Distance }} m</view>
		</view>
	</view>
</template>

<script>
	// 引入字符串处理函数
	import {
		stringify
	} from 'querystring';
	// 引入创建通用令牌函数
	const {
		createCommonToken
	} = require('@/key.js')
	
	// 产品ID和设备名称要替换成自己的
	const product_id = 'koQ9729Ag8';
	const device_name = 'car1';
	
	// Vue组件导出
	export default {
		// 数据部分
		data() {
			return {
				// 蜂鸣器状态
				Buzzer: false,
				// 系统状态
				SystemStatus: 0,
				// 车锁状态
				CarLock: false,
				// 车钥匙距离
				Distance: 0,
				// 刹车状态
				Brake: false,
				// 手刹状态
				ParkingBrake: false,
				// 接口请求token
				token: '',
				switch_flag: 0, //标记开关操作
			}
		},

		// 计算属性
		computed: {
			statusText() {
				const statusMap = {
					0: '正常',
					1: 'UWB异常',
					2: 'WiFi断连',
					3: 'CAN故障',
					4: '启动完成',
					5: 'MQTT断连'
				};
				return statusMap[this.SystemStatus] || '未知状态';
			}
		},

		// 页面加载时执行的钩子函数
		onLoad() {
			// 初始化token
			const params = {
				// 用户密钥和用户ID改成自己的，版本不用改
				author_key: 'GwBmohMRlYnuM4YTytrrSKpxlEBw8VgdCRBfsaVv8tBBdq17Gywxei+4GK6B1FOc',  //用户级秘钥
				version: '2022-05-01',
				user_id: '431899',  //用户ID
			}
			this.token = createCommonToken(params);
		},

		// 页面显示时执行的钩子函数
		onShow() {
			// 首次获取设备数据
			this.fetchDevData();
			// 定时（每3秒）获取设备数据
			setInterval(() => {
				this.fetchDevData();
			}, 3000)
		},

		// 方法部分
		methods: {
			// 获取设备数据的方法
			fetchDevData() {
				// 发送请求获取设备属性
				uni.request({
					url: 'https://iot-api.heclouds.com/thingmodel/query-device-property', 
					method: 'GET',
					data: {
						product_id: product_id,
						device_name: device_name
					},
					header: {
						'authorization': this.token // 自定义请求头信息
					},
					success: (res) => {
						// console.log(res.data);
						
						if(this.switch_flag == 1) {
							this.switch_flag = 0;
						} else {
							// 安全解析方式：确保返回了 data 并且数组有长度
							if (res.data.code === 0 && res.data.data && res.data.data.length > 0) {
								// 在数组中自动寻找各属性
								const buzzerItem = res.data.data.find(item => item.identifier === 'Buzzer');
								const statusItem = res.data.data.find(item => item.identifier === 'SystemStatus');
								const lockItem = res.data.data.find(item => item.identifier === 'CarLock');
								const distItem = res.data.data.find(item => item.identifier === 'Distance');
								const brakeItem = res.data.data.find(item => item.identifier === 'Brake');
								const parkItem = res.data.data.find(item => item.identifier === 'ParkingBrake');
								
								// 更新各属性状态
								if (buzzerItem) this.Buzzer = buzzerItem.value === 'true';
								if (statusItem) this.SystemStatus = Number(statusItem.value);
								if (lockItem) this.CarLock = lockItem.value === 'true';
								if (distItem) this.Distance = Number(Number(distItem.value).toFixed(1));
								if (brakeItem) this.Brake = brakeItem.value === 'true';
								if (parkItem) this.ParkingBrake = parkItem.value === 'true';
							}
						}
					},
					fail: (err) => {
						console.error("获取设备数据失败:", err);
					}
				});
			},

			// 蜂鸣器开关切换的方法
			onBuzzerSwitch(event) {
				console.log("用户操作开关，当前值:", event.detail.value);
				
				this.switch_flag = 1; //标记开关操作了
				
				let value = event.detail.value;
				// 发送请求更新设备属性
				uni.request({
					url: 'https://iot-api.heclouds.com/thingmodel/set-device-property', 
					method: 'POST',
					data: {
						product_id: product_id,
						device_name: device_name,
						params: {
							"Buzzer": value
						}
					},
					header: {
						'authorization': this.token // 自定义请求头信息
					},
					success: (res) => {
						if(res.data.code === 0) {
							console.log('BUZZER ' + (value ? 'ON' : 'OFF') + ' 下发成功!');
						} else {
							console.error('下发失败:', res.data);
						}
					},
					fail: (err) => {
						console.error("请求发送失败:", err);
					}
				});
			},

			// 车锁开关切换的方法
			onCarLockSwitch(event) {
				console.log("用户操作车锁，当前值:", event.detail.value);
				
				this.switch_flag = 1;
				
				let value = event.detail.value;
				uni.request({
					url: 'https://iot-api.heclouds.com/thingmodel/set-device-property', 
					method: 'POST',
					data: {
						product_id: product_id,
						device_name: device_name,
						params: {
							"CarLock": value
						}
					},
					header: {
						'authorization': this.token
					},
					success: (res) => {
						if(res.data.code === 0) {
							console.log('CARLOCK ' + (value ? 'UNLOCK' : 'LOCK') + ' 下发成功!');
						} else {
							console.error('下发失败:', res.data);
						}
					},
					fail: (err) => {
						console.error("请求发送失败:", err);
					}
				});
			},

			// 刹车按下
			onBrakeStart() {
				this.Brake = true;
				this.sendProperty('Brake', true);
			},

			// 刹车松开
			onBrakeEnd() {
				this.Brake = false;
				this.sendProperty('Brake', false);
			},

			// 手刹按下
			onParkingBrakeStart() {
				this.ParkingBrake = true;
				this.sendProperty('ParkingBrake', true);
			},

			// 手刹松开
			onParkingBrakeEnd() {
				this.ParkingBrake = false;
				this.sendProperty('ParkingBrake', false);
			},

			// 通用属性发送方法
			sendProperty(identifier, value) {
				this.switch_flag = 1;
				uni.request({
					url: 'https://iot-api.heclouds.com/thingmodel/set-device-property', 
					method: 'POST',
					data: {
						product_id: product_id,
						device_name: device_name,
						params: {
							[identifier]: value
						}
					},
					header: {
						'authorization': this.token
					},
					success: (res) => {
						if(res.data.code === 0) {
							console.log(identifier + ' 下发成功!');
						} else {
							console.error('下发失败:', res.data);
						}
					},
					fail: (err) => {
						console.error("请求发送失败:", err);
					}
				});
			}
		}
	}
</script>

<style>
	/* 整体页面容器样式 */
	.wrap {
		padding: 30rpx;
		/* 设置内边距为30像素 */
	}

	/* 设备区域样式 */
	.dev-area {
		display: flex;
		/* 使用弹性盒子布局 */
		justify-content: space-between;
		/* 在弹性容器中均匀分布子元素，两端对齐 */
		flex-wrap: wrap;
		/* 如果子元素溢出容器，则折叠到下一行 */
	}

	/* 设备卡片样式 */
	.dev-cart {
		height: 150rpx;
		/* 设置高度为150像素 */
		width: 320rpx;
		/* 设置宽度为320像素 */
		border-radius: 30rpx;
		/* 设置边框圆角为30像素 */
		margin-top: 30rpx;
		/* 设置上外边距为30像素 */
		display: flex;
		/* 使用弹性盒子布局 */
		justify-content: space-around;
		/* 在弹性容器中均匀分布子元素，两端对齐 */
		align-items: center;
		/* 在弹性容器中垂直居中对齐子元素 */
		box-shadow: 0 0 15rpx #ccc;
		/* 设置盒子阴影，颜色为灰色 */
	}

	/* 设备名称样式 */
	.dev-name {
		font-size: 20rpx;
		/* 设置字体大小为20像素 */
		text-align: center;
		/* 文本居中对齐 */
		color: #6d6d6d;
		/* 字体颜色为灰色 */
	}

	/* 设备图标样式 */
	.dev-logo {
		width: 70rpx;
		/* 设置宽度为70像素 */
		height: 70rpx;
		/* 设置高度为70像素 */
		margin-top: 10rpx;
		/* 设置上外边距为10像素 */
	}

	/* 系统状态区域样式 */
	.status-area {
		background-color: #fff;
		border-radius: 30rpx;
		padding: 30rpx;
		margin-bottom: 30rpx;
		box-shadow: 0 0 15rpx #ccc;
	}

	.status-title {
		font-size: 28rpx;
		color: #6d6d6d;
		margin-bottom: 20rpx;
	}

	.status-content {
		font-size: 32rpx;
		font-weight: bold;
		padding: 20rpx;
		border-radius: 15rpx;
		text-align: center;
	}

	.status-ok {
		background-color: #e8f5e9;
		color: #2b9939;
	}

	.status-err {
		background-color: #ffebee;
		color: #d32f2f;
	}

	/* 距离显示区域样式 */
	.distance-area {
		background-color: #fff;
		border-radius: 30rpx;
		padding: 30rpx;
		margin-top: 30rpx;
		box-shadow: 0 0 15rpx #ccc;
		text-align: center;
	}

	.distance-title {
		font-size: 28rpx;
		color: #6d6d6d;
		margin-bottom: 20rpx;
	}

	.distance-value {
		font-size: 48rpx;
		font-weight: bold;
		color: #1976d2;
	}

	/* 模拟按键区域样式 */
	.btn-area {
		margin-top: 30rpx;
	}

	.btn-title {
		font-size: 28rpx;
		color: #6d6d6d;
		margin-bottom: 20rpx;
	}

	.btn-row {
		display: flex;
		justify-content: space-between;
	}

	.btn-cart {
		height: 120rpx;
		width: 320rpx;
		border-radius: 30rpx;
		display: flex;
		justify-content: center;
		align-items: center;
		background-color: #fff;
		box-shadow: 0 0 15rpx #ccc;
		transition: all 0.2s;
	}

	.btn-active {
		background-color: #2b9939;
	}

	.btn-active .btn-name {
		color: #fff;
	}

	.btn-name {
		font-size: 28rpx;
		color: #6d6d6d;
		font-weight: bold;
	}
</style>