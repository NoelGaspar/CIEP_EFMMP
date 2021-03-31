#!/usr/bin/env python
# pylint: disable=W0613, C0116
# type: ignore[union-attr]
# This program is dedicated to the public domain under the CC0 license.

"""
Simple Bot to reply to Telegram messages.
First, a few handler functions are defined. Then, those functions are passed to
the Dispatcher and registered at their respective places.
Then, the bot is started and runs until we press Ctrl-C on the command line.
Usage:
Basic Echobot example, repeats messages.
Press Ctrl-C on the command line or send a signal to the process to stop the
bot.
"""

import logging
import paho.mqtt.client as mqtt
import json

import telegram
from telegram import Update
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, CallbackContext

from threading  import Thread
import datetime


MQTT_BROKER_ADDR  = "190.121.23.217"
MQTT_BROKER_PORT  = 1883
MQTT_PSW          = ""
MQTT_USSER        = ""
CHANNEL_SERVER_RX = "CIEP/tx"
CHANNEL_SERVER_TX = "CIEP/rx"

TOKEN = "1603181976:AAHr1qtE7K5cKyiYCXW3GfehU10EEklqmcI"
chat_id = "-520943913"
bot = telegram.Bot(token=TOKEN)


# Enable logging
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', level=logging.INFO
)

logger = logging.getLogger(__name__)


# Define a few command handlers. These usually take the two arguments update and
# context. Error handlers also receive the raised TelegramError object in error.
def start(update: Update, context: CallbackContext) -> None:
    """Send a message when the command /start is issued."""
    update.message.reply_text('Hi!')


def help_command(update: Update, context: CallbackContext) -> None:
    """Send a message when the command /help is issued."""
    update.message.reply_text('Help!')


def echo(update: Update, context: CallbackContext) -> None:
    """Echo the user message."""
    update.message.reply_text(update.message.text)

def send_msg_bot(msg) :
    bot.send_message(chat_id= chat_id, text=msg)

def connected(client, userdata, flag , rc):
    print("server connected")
    client.subscribe(CHANNEL_SERVER_RX)
    client.publish(CHANNEL_SERVER_TX, "HELLO WORD")

def rx_callback(client, userdata,msg):
    print("mensaje recibido : ", end="")
    msg_rx = msg.payload.decode("utf-8") 
    print(msg_rx)
    send_msg_bot(msg_rx)
    #update.message.reply_text(msg_rx)
  

def bot_run():
    """Start the bot."""
    # Create the Updater and pass it your bot's token.
    updater = Updater(TOKEN)

    # Get the dispatcher to register handlers
    dispatcher = updater.dispatcher

    # on different commands - answer in Telegram
    dispatcher.add_handler(CommandHandler("start", start))
    dispatcher.add_handler(CommandHandler("help", help_command))

    # on noncommand i.e message - echo the message on Telegram
    dispatcher.add_handler(MessageHandler(Filters.text & ~Filters.command, echo))

    # Start the Bot
    updater.start_polling()

    # Run the bot until you press Ctrl-C or the process receives SIGINT,
    # SIGTERM or SIGABRT. This should be used most of the time, since
    # start_polling() is non-blocking and will stop the bot gracefully.
    updater.idle()

def mqtt_run():
    print("connecting to mqtt client")  
    client = mqtt.Client("server_test")
    client.username_pw_set(MQTT_USSER, MQTT_PSW)
    client.on_message = rx_callback      #attach to callback function
    client.on_connect = connected        #attach initial subscribe test
    #client connect
    client.connect(MQTT_BROKER_ADDR,MQTT_BROKER_PORT,60)
    client.tls_set()

    while True:
        client.loop()

    
if __name__ == '__main__':
    bot_thread=Thread(target=mqtt_run)
    bot_thread.setDaemon(True)
    bot_thread.start()
    bot_run()