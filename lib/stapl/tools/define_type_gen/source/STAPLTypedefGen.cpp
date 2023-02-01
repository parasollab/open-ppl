/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//Currently a stand alone tool - may be able to integrate into a full compiler
//Clang's FixItRecompile class may give some ideas on how to do this

#include "clang/AST/AST.h"
#include "clang/AST/Decl.h"
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/ASTConsumers.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendActions.h"
#include "clang/Rewrite/Core/Rewriter.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Frontend/VerifyDiagnosticConsumer.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FileSystem.h"
#include <unordered_map>
#include <sstream>
#include <string>
#include <vector>

using namespace clang;
using namespace clang::driver;
using namespace clang::tooling;

/*** COMMMAND LINE OPTIONS */

static llvm::cl::OptionCategory STAPLTypedefGenCategory("STAPL Typedef Generator");
static llvm::cl::opt<std::string> Classes("ignore-classes", llvm::cl::cat(STAPLTypedefGenCategory), llvm::cl::desc("Specify comma separated list of qualified class names to ignore"));
static llvm::cl::opt<bool> NewFile("newfile", llvm::cl::cat(STAPLTypedefGenCategory), llvm::cl::desc("Write any changed files to a new file rather than in place"));
static llvm::cl::opt<bool> Regen("regen", llvm::cl::cat(STAPLTypedefGenCategory), llvm::cl::desc("Regenerate all typedef functions"));
static llvm::cl::opt<std::string> ProjDir("dir", llvm::cl::cat(STAPLTypedefGenCategory), llvm::cl::desc("Specify an absolute project directoy. Only files in the specified directory (and subdirectories) will be scanned and edited"));
static llvm::cl::opt<std::string> NewFilePrefix("newfilePrefix", llvm::cl::cat(STAPLTypedefGenCategory), llvm::cl::desc("Prefix to be appended to generated files. Can include path from working directory. Default is 'edited_'."));


/*** GLOBAL VARIABLES */

std::unordered_map<std::string, std::string> visited;    //global so that constant when each file has diff frontend action
const std::string dummyDiogComment = "// expected-error 0-1 {{dummy diagnostic to fool verifying consumer}}"; //dummy comment to "trick" verifying diagnostic consumer


/*** FIRST COMPILER PASS
*** First compiler pass; adds a define_type declaration to every class. We ignore the diagnostic messages produced
*** by this pass since anything unrelated to not having a define_type will be printed in the main pass. We should only
*** run this pass if newfile is not set; if newfile is set the second pass will produce errors anyway*/

/*** visitors allows us to iterate over various nodes and do an action for each one */
class STAPLTypedefGenFirstASTVisitor : public RecursiveASTVisitor<STAPLTypedefGenFirstASTVisitor> {
public:

  /* constructor */
  STAPLTypedefGenFirstASTVisitor(Rewriter &R) : rw(R) {}

  /* function that sets the class blacklist for the pass */
  void setClasses(std::string class_list){

    //clear vector
    classes = std::unordered_map<std::string, std::string>();

    //parse class list and add to vector
    std::string current;
    for(size_t i=0; i<class_list.length(); ++i){
      if(class_list.at(i) == ','){
        classes.insert({current, current});
        current = "";
      }
      else{
        current += class_list.at(i);
      }
    }
    classes.insert({current, current});
  }

  /* functionthat is called from the ASTconsumer, does an action each time a class or struct is found */
  bool VisitCXXRecordDecl(CXXRecordDecl* in_cl){
    if(visited.find(in_cl->getQualifiedNameAsString()) != visited.end()){
        //already visisted, move on
        return true;
    }
    if(classes.find(in_cl->getQualifiedNameAsString()) != classes.end()){
      //in exclusion list, ignore
      return true;
    }
    if(!in_cl->hasDefinition()){
        //no definition, move on
        return true;
    }

    //parse class location and make sure it's in provided directory
    CXXRecordDecl* cl = in_cl->getDefinition();
    if(!cl->getLocation().isFileID()){
      //not from a file, from a macro or something, ignore
      return true;
    }
    std::string class_loc = getAbsolutePath(StringRef(cl->getLocation().printToString(rw.getSourceMgr())));
    std::string proj_dir = ProjDir.getValue();
    if(proj_dir.length() >0 && proj_dir.at(proj_dir.length()-1) != '/'){
      proj_dir.push_back('/');
    }
    if(proj_dir.length() >0 && proj_dir.length() >= class_loc.length()){
      //file can't be in directory because the directory name is longer than the full name of the string with line numbers
      return true;
    }
    class_loc.resize(proj_dir.length());
    if(proj_dir.length() >0 && proj_dir != class_loc){
      //this class is located in a file location that doesn't start with project directory
      //hence it cannot be a subdirectory of the project directory. ignore
      return true;
    }

    //if we get here, then we haven't processed this class yet
    //add it to the list of processed classes
    visited.insert({cl->getQualifiedNameAsString(), cl->getQualifiedNameAsString()});

    //check to see if there is already a defined define_type function
    for(CXXRecordDecl::method_iterator it = cl->method_begin(); it != cl->method_end(); ++it){

      //do we already have a define type?
      if(it->getNameAsString() == "define_type" && it->getNumParams()==1){
        //check args
        QualType paramType = (*it->param_begin())->getType();
        if(!paramType.isConstQualified() && paramType.getTypePtr()->isReferenceType()
           && paramType.getNonReferenceType().getTypePtr()->getAsCXXRecordDecl()
           && paramType.getNonReferenceType().getTypePtr()->getAsCXXRecordDecl()->getQualifiedNameAsString() == "stapl::typer"){
          //decl already exists, move on
          return true;
        }
      }
    }

    //add declaration to rewriter
    if(rw.isRewritable(cl->getLocEnd())) rw.InsertTextBefore(cl->getLocEnd(), "  void define_type(stapl::typer& t);\n");
    
    //keep processing tree
    return true;
  }

private:
    std::unordered_map<std::string, std::string> classes;
    Rewriter &rw;
};

/*** ASTConsumer actually reads the file. Called from FrontendAction */
class STAPLTypedefGenFirstASTConsumer : public ASTConsumer {
public:
    /* constructor */
    STAPLTypedefGenFirstASTConsumer(Rewriter &R) : visitor(R), rw(R) {}

    /* function called for every file after parsing */
    virtual void HandleTranslationUnit(ASTContext &Context) {
        SourceManager& sm = rw.getSourceMgr();

        //add dummy comment to the top of every file to fool verifying consumer in second pass
        rw.InsertTextAfter(sm.getLocForStartOfFile(sm.getMainFileID()), StringRef(dummyDiogComment + "\n"));

        //run astvisitor to actually add declarations
        visitor.setClasses(Classes.getValue());
        visitor.TraverseDecl(Context.getTranslationUnitDecl());
    }

private:
  STAPLTypedefGenFirstASTVisitor visitor;
  Rewriter& rw;
};

/*** A Frontend Action is made for every pass and started from ClangTool::run() */
class STAPLTypedefGenFirstFrontendAction : public ASTFrontendAction {
public:
  
  /* this is called after the compiler instance and invocation are set, but before processing the file */
  bool BeginSourceFileAction(CompilerInstance &CI, StringRef InFile) override{
    llvm::outs() << "Processing " << InFile.str() <<" in first pass...\n";
    return true;
  }

  /* this is called after processing the file with the ASTConsumer */
  void EndSourceFileAction() override {
    SourceManager &SM = rw.getSourceMgr();

    //assume regen is set
    rw.overwriteChangedFiles();

    //need to clear FileManager cache so that future reads to written files read the entire file and don't produce errors
    FileManager& fm = SM.getFileManager();
    for(Rewriter::buffer_iterator i= rw.buffer_begin(); i!=rw.buffer_end(); ++i){
      fm.invalidateCache(SM.getFileEntryForID(i->first));
    }
  }

  /* This is what is called by Tool::run() to actually run the ASTConsumer */
  std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &CI, StringRef file) override {
    rw.setSourceMgr(CI.getSourceManager(), CI.getLangOpts());
    return llvm::make_unique<STAPLTypedefGenFirstASTConsumer>(rw);
  }
private:
  Rewriter rw;
};


/*** MAIN COMPILER PASS
 *** This is the pass that actually does most of the work. It defines a define_type function inline for every
 *** class (with some exceptions that depend on the user options). It uses a verifying consumer to ignore static
 *** asserts added by the program. */

/*** visitors allows us to iterate over various nodes and do an action for each one */
class STAPLTypedefGenASTVisitor : public RecursiveASTVisitor<STAPLTypedefGenASTVisitor> {
public:

  /* constructor */
  STAPLTypedefGenASTVisitor(Rewriter &R) : rw(R) {}

  /* set the class blacklist for the visitor */
  void setClasses(std::string class_list){

    //clear vector
    classes = std::unordered_map<std::string, std::string>();

    //parse class list and add to vector
    std::string current;
    for(size_t i=0; i<class_list.length(); ++i){
      if(class_list.at(i) == ','){
        classes.insert({current, current});
        current = "";
      }
      else{
        current += class_list.at(i);
      }
    }
    classes.insert({current, current});
  }

  /* functionthat is called from the ASTconsumer, does an action each time a class or struct is found */
  bool VisitCXXRecordDecl(CXXRecordDecl* in_cl){
    if(visited.find(in_cl->getQualifiedNameAsString()) != visited.end()){
        //already visisted, move on
        return true;
    }
    if(classes.find(in_cl->getQualifiedNameAsString()) != classes.end()){
      //in exclusion list, ignore
      return true;
    }
    if(!in_cl->hasDefinition()){
        //no definition, move on
        return true;
    }

    //parse class location and make sure it's in provided directory
    CXXRecordDecl* cl = in_cl->getDefinition();
    if(!cl->getLocation().isFileID()){
      //not from a file, from a macro or something, ignore
      return true;
    }
    std::string class_loc = getAbsolutePath(StringRef(cl->getLocation().printToString(rw.getSourceMgr())));
    std::string proj_dir = ProjDir.getValue();
    if(proj_dir.length() >0 && proj_dir.at(proj_dir.length()-1) != '/'){
      proj_dir.push_back('/');
    }
    if(proj_dir.length() >0 && proj_dir.length() >= class_loc.length()){
      //file can't be in directory because the directory name is longer than the full name of the string with line numbers
      return true;
    }
    class_loc.resize(proj_dir.length());
    if(proj_dir.length() >0 && proj_dir != class_loc){
      //this class is located in a file location that doesn't start with project directory
      //hence it cannot be a subdirectory of the project directory. ignore
      return true;
    }

    //if we get here, then we haven't processed this class yet
    //add it to the list of processed classes
    visited.insert({cl->getQualifiedNameAsString(), cl->getQualifiedNameAsString()});

    //check to see if there is already a defined define_type function
    for(CXXRecordDecl::method_iterator it = cl->method_begin(); it != cl->method_end(); ++it){

      //do we already have a define type?
      if(it->getNameAsString() == "define_type" && it->getNumParams()==1){
        //check args
        QualType paramType = (*it->param_begin())->getType();
        if(!paramType.isConstQualified() && paramType.getTypePtr()->isReferenceType()
           && paramType.getNonReferenceType().getTypePtr()->getAsCXXRecordDecl()
           && paramType.getNonReferenceType().getTypePtr()->getAsCXXRecordDecl()->getQualifiedNameAsString() == "stapl::typer"){

          if(!Regen.getValue() && it->hasBody()){
            //already have one, and it is already defined, and we're not regenerating
            //do not process
            return true;
          }
          else if(it->hasBody()){
            //delete declaration and definition separately
            if(it->getLocation() != it->getDefinition()->getLocation()){
              rw.RemoveText(it->getSourceRange());
              rw.RemoveText(it->getDefinition()->getSourceRange());
            }
            //only need to delete current
            else{
              rw.RemoveText(it->getSourceRange());
            }
          }
          else{
            //declared but not defined go ahead and delete declaration since it will be readded later
            rw.RemoveText(SourceRange(it->getLocStart(), it->getLocEnd().getLocWithOffset(1)));
          }
        }
      }
    }

    std::stringstream ss;
    //build function definition
    ss<<"  void define_type(stapl::typer& t){\n";
    ss<<"    //make sure this method is public\n    // if stapl::typer is not found, please include <stapl/runtime/serialization.hpp>\n\n";
    ss<<"    //put any computations here if necessary\n";

    //handle base classes first
    for(CXXRecordDecl::base_class_iterator it = cl->bases_begin(); it!=cl->bases_end(); ++it){

        CXXRecordDecl* baseDecl = it->getType().getTypePtr()->getAsCXXRecordDecl();
        if(baseDecl){
            ss<<"    t.base<"<<baseDecl->getNameAsString() <<">(*this);" << "\n";
        }
    }

    //handle members second
    for(RecordDecl::field_iterator it = cl->field_begin(); it != cl->field_end(); ++it){
        if(it->getType().getTypePtr()->isPointerType()){
            ss<<"    static_assert(false, \"Adjust t.member call below appropriately\"); // expected-error {{static_assert failed \"Adjust t.member call below appropriately\"}}\n";
            ss<<"    t.member(" << it->getNameAsString() << "/*, insert member storing size if needed */ );\n";
            ss<<"    // Use t.pointer_to_member("<<it->getNameAsString()
              <<", U* originalMember, size_t offset) if the pointer is to already serialized member\n";
        }
        else{
            ss<<"    t.member(" << it->getNameAsString() << ");\n";
        }
    }

    //finish function
    ss<<"  }\n";

    //add to rewriter
    if(rw.isRewritable(cl->getLocEnd())) rw.InsertTextBefore(cl->getLocEnd(), ss.str());

    //keep processing tree
    return true;
  }

private:
    std::unordered_map<std::string, std::string> classes;
    Rewriter &rw;
};

/*** ASTConsumer actually reads the file. Called from FrontendAction */
class STAPLTypedefGenASTConsumer : public ASTConsumer {
public:

    /* constructor */
    STAPLTypedefGenASTConsumer(Rewriter &R) : visitor(R), rw(R) {}

    /* function called for every file after parsing */
    virtual void HandleTranslationUnit(ASTContext &Context) {
      //traverse tree and do rewriting
      visitor.setClasses(Classes.getValue());
      visitor.TraverseDecl(Context.getTranslationUnitDecl());

      //get rid of dummyDiagComments
      if(!NewFile.getValue()){
        auto comments = Context.getRawCommentList().getComments();
        for(RawComment* comment : comments){
          if(comment->getRawText(Context.getSourceManager()).str() == dummyDiogComment){
            rw.RemoveText(comment->getSourceRange());
          }
        }
      }
    }

private:
  STAPLTypedefGenASTVisitor visitor;
  Rewriter &rw;
};

/*** A Frontend Action is made for every pass and started from ClangTool::run() */
class STAPLTypedefGenFrontendAction : public ASTFrontendAction {
public:

  /* This is called before the file is processed, and before the compiler instance and invocation are completely set up */
  bool BeginInvocation(CompilerInstance& CI) override{
    if(!NewFile.getValue()){
        verifyingConsumer = new VerifyDiagnosticConsumer(CI.getDiagnostics());
        CI.getDiagnostics().setClient(verifyingConsumer);
      }
      return true;  
  }

  /* this is called after the compiler instance and invocation are set, but before processing the file */
  bool BeginSourceFileAction(CompilerInstance &CI, StringRef InFile) override{
    llvm::outs() << "Processing " << InFile.str() <<" in main pass...\n";
    return true;
  }

  /* this is called after processing the file with the ASTConsumer */
  void EndSourceFileAction() override {
    SourceManager &SM = rw.getSourceMgr();

    if(NewFile.getValue()){

      std::string prefix = NewFilePrefix.getValue();
      if(prefix == "") prefix = "edited_";

      for(Rewriter::buffer_iterator i= rw.buffer_begin(); i!=rw.buffer_end(); ++i){
        std::string filename = SM.getFilename(SM.getLocForStartOfFile(i->first)).str();

        //remove path from filename
        int loc = filename.length()-1;
        for(; loc>=0; --loc){
          if(filename.at(loc)=='/'){
            break;
          }
        }
        filename.erase(0, loc+1);
        filename = prefix + filename;
        std::error_code errs;
        llvm::raw_fd_ostream out(llvm::StringRef(filename),errs,llvm::sys::fs::OpenFlags::F_None);

        rw.getEditBuffer(i->first).write(out);
      }
    }
    else{
      rw.overwriteChangedFiles();

      //need to clear FileManager cache so that future reads to written files read the entire file and don't produce errors
      FileManager& fm = SM.getFileManager();
      for(Rewriter::buffer_iterator i= rw.buffer_begin(); i!=rw.buffer_end(); ++i){
        fm.invalidateCache(SM.getFileEntryForID(i->first));
      }
    }
  }

  /* This is what is called by Tool::run() to actually run the ASTConsumer */
  std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &CI, StringRef file) override {
    rw.setSourceMgr(CI.getSourceManager(), CI.getLangOpts());
    return llvm::make_unique<STAPLTypedefGenASTConsumer>(rw);
  }
private:
  Rewriter rw;
  VerifyDiagnosticConsumer* verifyingConsumer; //use a pointer so that it can be unused at start
};

/*** MAIN FUNCTION
*** Sets up a ClangTool that parses commandline options then runs both
*** frontend actions */

int main(int argc, const char **argv) {
  CommonOptionsParser op(argc, argv, STAPLTypedefGenCategory);
  ClangTool Tool(op.getCompilations(), op.getSourcePathList());

  llvm::outs()<<"\n";

  if(!NewFile.getValue() && ProjDir.getValue() == ""){
    llvm::errs()<<"You must specify a project directory if '-newfile' is not specified.\n";
    return 1;
  }

  if(!NewFile.getValue() && NewFilePrefix.getValue() != ""){
    llvm::errs()<<"-newfilePrefix should not be specified if -newfile is not specified\n";
    return 1;
  }

  for(std::string file : op.getSourcePathList()){
    //add an option that allows the tool to read all comments and not just doxygen style comments; needed for verifying diag consumer
    Tool.appendArgumentsAdjuster(getInsertArgumentAdjuster("-fparse-all-comments", ArgumentInsertPosition::BEGIN));
  }

  //set to ignore diagnostics and run first pass, but only if we're writing in place
  IgnoringDiagConsumer igDiagCons;
  Tool.setDiagnosticConsumer(&igDiagCons);
  if(!NewFile.getValue()){
    Tool.run(newFrontendActionFactory<STAPLTypedefGenFirstFrontendAction>().get());
  }

  //clear visited list for second action
  visited = std::unordered_map<std::string, std::string>();

  //set diagnostic consumer for second pass
  Tool.setDiagnosticConsumer(nullptr);
  int result = Tool.run(newFrontendActionFactory<STAPLTypedefGenFrontendAction>().get());

  llvm::outs()<<"\n";

  return result;
}
