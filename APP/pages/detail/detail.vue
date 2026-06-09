<template>
	<!-- 整体布局 -->
	<view class="wrap">
		<!-- 设备区域 -->
		<view class="dev-area">

			<!-- 设备卡片 - 蜂鸣器 -->
			<view class="dev-cart">
				<view>
					<view class="dev-name">蜂鸣器</view>
					<!-- 蜂鸣器图标 -->
					<image class="dev-logo" src="../../static/buzzer.png" mode=""></image>
				</view>
				<!-- 蜂鸣器开关 -->
				<switch :checked="Buzzer" @change="onBuzzerSwitch" color="#2b9939" />
			</view>

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
				// 接口请求token
				token: '',
				switch_flag: 0, //标记开关操作
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
								// 在数组中自动寻找 identifier 为 'Buzzer' 的那一项
								const buzzerItem = res.data.data.find(item => item.identifier === 'Buzzer');
								
								// 如果找到了，就更新开关状态
								if (buzzerItem) {
									this.Buzzer = buzzerItem.value === 'true';
								}
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
</style>