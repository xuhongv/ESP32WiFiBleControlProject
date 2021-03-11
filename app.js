//app.js
App({
  buf2hex: function (buffer) {
    return (new Uint8Array(buffer))
  },
  buf2string: function (buffer) {
    var arr = Array.prototype.map.call(new Uint8Array(buffer), x => x)
    var str = ''
    for (var i = 0; i < arr.length; i++) {
      str += String.fromCharCode(arr[i])
    }
    return str
  },
  onLaunch: function () {
    this.globalData.SystemInfo = wx.getSystemInfoSync()
    //console.log(this.globalData.SystemInfo)
  },
  globalData: {
    SystemInfo: {}
  }
})