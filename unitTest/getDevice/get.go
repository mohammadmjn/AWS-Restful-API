package main

import (
	"aws-restful-api/device"
	"encoding/json"
	"net/http"
	"os"
	"strings"

	"github.com/aws/aws-lambda-go/events"
	"github.com/aws/aws-lambda-go/lambda"
	"github.com/aws/aws-sdk-go/aws"
	"github.com/aws/aws-sdk-go/aws/session"
	"github.com/aws/aws-sdk-go/service/dynamodb"
	"github.com/aws/aws-sdk-go/service/dynamodb/dynamodbattribute"
	"github.com/aws/aws-sdk-go/service/dynamodb/dynamodbiface"
)

var dynamoDBSvc dynamodbiface.DynamoDBAPI

func init() {
	sess := session.Must(session.NewSessionWithOptions(session.Options{SharedConfigState: session.SharedConfigEnable}))
	dynamoDBSvc = dynamodb.New(sess)
}

func GetDevice(deviceId string, dynamoIfaceSvc dynamodbiface.DynamoDBAPI) (*dynamodb.GetItemOutput, error) {
	tableName := aws.String(os.Getenv("TABLE_NAME"))
	input := &dynamodb.GetItemInput{
		TableName: tableName,
		Key: map[string]*dynamodb.AttributeValue{
			"id": {
				S: aws.String(deviceId),
			},
		},
	}
	operOutput, err := dynamoIfaceSvc.GetItem(input)
	return operOutput, err
}

func GetHandler(request events.APIGatewayProxyRequest) (events.APIGatewayProxyResponse, error) {
	if request.HTTPMethod != "GET" {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: Invalid http method",
		}, nil
	}

	deviceId := ""
	splitPath := strings.Split(request.Path, "/")
	if len(splitPath) == 4 && strings.HasPrefix(request.Path, "/api/devices/") {
		deviceId = splitPath[len(splitPath)-1]
	} else {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusForbidden,
			Body:       "403 Forbidden Access: URL path is incorrect",
		}, nil
	}
	if deviceId == "" {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: ID of device is missed in path",
		}, nil
	}

	result, err2 := GetDevice(deviceId, dynamoDBSvc)
	if err2 != nil {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusInternalServerError,
			Body:       "500 Internal Server Error: The server encountered an internal error or misconfiguration to connect to database",
		}, nil
	}

	if len(result.Item) == 0 {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusNotFound,
			Body:       "404 Not Found: Requested item does not exist in the database",
		}, nil
	}

	device := device.Device{}
	err2 = dynamodbattribute.UnmarshalMap(result.Item, &device)

	if err2 != nil {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusInternalServerError,
			Body:       "500 Internal Server Error",
		}, nil
	}

	deviceJson, _ := json.Marshal(device)

	return events.APIGatewayProxyResponse{
		StatusCode: http.StatusOK,
		Body:       string(deviceJson),
	}, nil
}

func main() {
	lambda.Start(GetHandler)
}
