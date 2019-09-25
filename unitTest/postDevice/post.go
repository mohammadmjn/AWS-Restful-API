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

func PutDevice(device map[string]*dynamodb.AttributeValue, dynamoIfaceSvc dynamodbiface.DynamoDBAPI) (*dynamodb.PutItemOutput, error) {
	tableName := aws.String(os.Getenv("TABLE_NAME"))
	input := &dynamodb.PutItemInput{
		Item:      device,
		TableName: tableName,
	}
	operOutput, err := dynamoIfaceSvc.PutItem(input)
	return operOutput, err
}

func ValidateDeviceInfo(device device.Device) []string {
	missedField := []string{}
	if device.Id == "" {
		missedField = append(missedField, "'id' ")
	}

	if device.DeviceModel == "" {
		missedField = append(missedField, "'device model' ")
	}

	if device.Name == "" {
		missedField = append(missedField, "'name' ")
	}

	if device.Note == "" {
		missedField = append(missedField, "'note' ")
	}

	if device.Serial == "" {
		missedField = append(missedField, "'serial' ")
	}
	return missedField
}

func PostHandler(request events.APIGatewayProxyRequest) (events.APIGatewayProxyResponse, error) {
	if request.HTTPMethod != "POST" {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: Invalid http method",
		}, nil
	}
	if request.Headers["Content-Type"] != "application/json" {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: Invalid content type. It is not in a JSON format",
		}, nil
	}

	splitPath := strings.Split(request.Path, "/")
	if len(splitPath) != 3 || !strings.HasPrefix(request.Path, "/api/devices") {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusForbidden,
			Body:       "403 Forbidden: You don't have permission to access " + request.Path,
		}, nil
	}

	if len(request.Body) == 0 {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: Invalid JSON request. All fields are empty",
		}, nil
	}

	device := device.Device{}
	err := json.Unmarshal([]byte(request.Body), &device)

	if err != nil {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: Invalid JSON format",
		}, nil
	}

	missedFields := ValidateDeviceInfo(device)
	if len(missedFields) != 0 {
		messageMissedFields := ""
		for _, fields := range missedFields {
			messageMissedFields += fields
		}
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusBadRequest,
			Body:       "400 Bad Request: No value is specified for field(s) " + messageMissedFields + "of device in json request",
		}, nil
	}

	deviceItem, _ := dynamodbattribute.MarshalMap(device)
	_, err = PutDevice(deviceItem, dynamoDBSvc)
	if err != nil {
		return events.APIGatewayProxyResponse{
			StatusCode: http.StatusInternalServerError,
			Body:       "500 Internal Server Error: The server encountered an internal error or misconfiguration to connect to database",
		}, nil
	}

	responseBody, _ := json.Marshal(device)
	return events.APIGatewayProxyResponse{
		StatusCode: http.StatusCreated,
		Body:       string(responseBody),
	}, nil
}

func main() {
	lambda.Start(PostHandler)
}
