/********************************************************************

  Filename:   MyJson

  Description:MyJson

  Version:  1.0
  Created:  8:4:2015   11:01
  Revison:  none
  Compiler: gcc vc

  Author:   wufan, love19862003@163.com

  Organization:
*********************************************************************/
#ifndef __MyJson_H__
#define __MyJson_H__
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

  namespace Utility {
     class MyJson
     {
     private:
       MyJson(const MyJson&) = delete;
       MyJson& operator = (const MyJson&) = delete;
     public:
       explicit MyJson(const std::string& file) : m_tree(){
           try{
               boost::property_tree::json_parser::read_json(file, m_tree);
           }catch (std::exception& e)
           {
               std::cout<< e.what() << std::endl;
           }
           
       }
       explicit MyJson(std::stringstream& ss) : m_tree() {
           try{
               boost::property_tree::json_parser::read_json(ss, m_tree);
           } catch(std::exception& e){
               std::cout << e.what() << std::endl;
           }
       }
       explicit MyJson(const boost::property_tree::ptree& tree) : m_tree(tree) {

       }
       explicit MyJson() : m_tree(){

       }

       std::string toString() {
         std::stringstream ss;
         boost::property_tree::json_parser::write_json(ss, m_tree);
         return ss.str();
       }

       template< typename  T>
       T get(const std::string& path, const T& t = T()) const {
         return m_tree.get(path, t);
       }

       template < typename T>
       void getArray(const std::string& path, std::vector<T>& container) const {
         using boost::property_tree::ptree;
         try{
           BOOST_FOREACH(const ptree::value_type& v, m_tree.get_child(path)){
             container.push_back(boost::lexical_cast<T>(v.second.data()));
           }
         }catch (...)
         {
         }            
       }

       template < typename T>
       void getGroup(const std::string& path,
                     std::function<T&(const MyJson&)> fun) const{
         using boost::property_tree::ptree;
         fun(MyJson(m_tree.get_child(path)));
       }

       template < typename T>
       void getGroup(const std::string& path, 
                     std::vector<T>& container, 
                     std::function<T(const MyJson&)> fun) const {
         using boost::property_tree::ptree;
         BOOST_FOREACH(const ptree::value_type& v, m_tree.get_child(path)) {
           MyJson t(v.second);
           container.push_back(fun(t));
         }
       }
       template < typename T>
       void set(const std::string& node, const T& t) {
         m_tree.put(node, t);
       }
       template< typename T>
       void addArray(const std::string& node, std::vector<T>& arr) {
         Utility::MyJson tree;
         for (auto& t : arr){
           tree.m_tree.add("", t);
         }
         m_tree.push_back(std::make_pair(node, tree.m_tree));
       }

       void addTreeArray(const std::string& node, std::vector<MyJson>& arr) {
         Utility::MyJson tree;
         for(auto& t : arr) {
           tree.m_tree.push_back(std::make_pair("", t.m_tree));
         }
         m_tree.push_back(std::make_pair(node, tree.m_tree));
       }
       
       bool saveToFile(const std::string& file_name) {
         using boost::property_tree::ptree;
         try {
           write_json(file_name, m_tree);
           return true;
         } catch(...) {
           std::cout << "write json file error : " << file_name << std::endl;
           return false;
         }
       }
     protected:
       boost::property_tree::ptree m_tree;
     };
  }
#endif