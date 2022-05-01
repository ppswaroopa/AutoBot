#!/usr/bin/env python3

# Implemented with reference from https://gist.github.com/manhay212/987c32b634927a4388ad7aba6b7bcbbf

import requests
import socket
import datetime

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def telegram_bot_sendtext(bot_message):
    
    bot_token = 'TOKEN' # Replace it with your token
    bot_chatID = 'CHATID' # Replace it with your chat ID
    send_text = 'https://api.telegram.org/bot' + bot_token + '/sendMessage?chat_id=' + bot_chatID + '&parse_mode=Markdown&text=' + bot_message

    response = requests.get(send_text)

    return response.json()
    

test = telegram_bot_sendtext("IP Address at time "+str(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))+" is "+str(get_ip()))
