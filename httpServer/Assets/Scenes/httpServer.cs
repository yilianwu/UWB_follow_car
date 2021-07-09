using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using System;
using System.IO;
using System.Net;
using System.Threading;

public class httpServer : MonoBehaviour
{
    private HttpListener listener;
	private Thread listenerThread;

    // Start is called before the first frame update
    void Start()
    {
        listener = new HttpListener ();
		listener.Prefixes.Add ("http://*:5555/");
		// listener.Prefixes.Add ("http://127.0.0.1:5555/");
		listener.AuthenticationSchemes = AuthenticationSchemes.Anonymous;
		listener.Start ();

		listenerThread = new Thread (startListener);
		listenerThread.Start ();
		Debug.Log ("Server Started");
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void startListener ()
	{
		while (true) {               
			var result = listener.BeginGetContext (ListenerCallback, listener); //看看是否有人傳request?
			result.AsyncWaitHandle.WaitOne ();
		}
	}

	private void ListenerCallback (IAsyncResult result)
	{				
		var context = listener.EndGetContext (result);		

		Debug.Log ("Method: " + context.Request.HttpMethod);
		Debug.Log ("LocalUrl: " + context.Request.Url.LocalPath);

		if (context.Request.Url.LocalPath == "/build") { 
			// && 確認上一個蓋房子動作完成了
				//building show 出現建築
			context.Response.StatusCode = 202;
		}
		else if (context.Request.Url.LocalPath == "/gpsreset") { 
			// GPS Reset
			context.Response.StatusCode = 202;
		}
		else
		{
			context.Response.StatusCode = 404;
		}

		if (context.Request.QueryString.AllKeys.Length > 0)
			foreach (var key in context.Request.QueryString.AllKeys) {
				Debug.Log ("Key: " + key + ", Value: " + context.Request.QueryString.GetValues (key) [0]);
			}

		if (context.Request.HttpMethod == "POST") {	
			Thread.Sleep (1000);
			var data_text = new StreamReader (context.Request.InputStream, 
				                context.Request.ContentEncoding).ReadToEnd ();
			Debug.Log (data_text);
		}

		context.Response.Close ();
	}
}
